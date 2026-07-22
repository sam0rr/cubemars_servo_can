"""Quality checks for executable Python examples in project documentation."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def _fence_start(line: str) -> tuple[str, int, int, bool] | None:
    """Return the structure and Python status of a Markdown opening fence."""
    content = line.rstrip("\r\n")
    stripped = content.lstrip(" ")
    indentation = len(content) - len(stripped)
    if indentation > 3 or not stripped:
        return None

    marker = stripped[0]
    if marker not in ("`", "~"):
        return None

    fence_width = len(stripped) - len(stripped.lstrip(marker))
    if fence_width < 3:
        return None

    info_string = stripped[fence_width:].strip()
    language = info_string.split(maxsplit=1)[0] if info_string else ""
    return marker, fence_width, indentation, language == "python"


def _is_closing_fence(line: str, *, marker: str, minimum_width: int) -> bool:
    """Return whether a line closes the active Markdown code fence."""
    content = line.rstrip("\r\n")
    stripped = content.lstrip(" ")
    indentation = len(content) - len(stripped)
    if indentation > 3 or not stripped.startswith(marker):
        return False

    fence_width = len(stripped) - len(stripped.lstrip(marker))
    return fence_width >= minimum_width and not stripped[fence_width:].strip()


def _strip_fence_indentation(line: str, indentation: int) -> str:
    """Remove the opening fence indentation from one example line."""
    leading_spaces = len(line) - len(line.lstrip(" "))
    return line[min(indentation, leading_spaces) :]


def _python_examples(path: Path) -> list[str]:
    """Extract fenced Python examples from one Markdown document."""
    examples: list[str] = []
    active_fence: tuple[str, int, int, bool] | None = None
    example_lines: list[str] = []

    for line in path.read_text(encoding="utf-8").splitlines(keepends=True):
        if active_fence is None:
            active_fence = _fence_start(line)
            if active_fence is not None and active_fence[3]:
                example_lines = []
            continue

        marker, minimum_width, indentation, is_python = active_fence
        if _is_closing_fence(
            line,
            marker=marker,
            minimum_width=minimum_width,
        ):
            if is_python:
                examples.append("".join(example_lines))
            active_fence = None
            continue
        if is_python:
            example_lines.append(_strip_fence_indentation(line, indentation))

    if active_fence is not None and active_fence[3]:
        examples.append("".join(example_lines))
    return examples


def _run_ruff_lint(
    source: str, repository_root: Path
) -> subprocess.CompletedProcess[str]:
    """Run the configured Ruff linter against one documentation example."""
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "ruff",
            "check",
            "--no-cache",
            "--stdin-filename",
            "documentation_example.py",
            "-",
        ],
        cwd=repository_root,
        input=source,
        capture_output=True,
        text=True,
        check=False,
    )


def _run_ruff_format(
    source: str,
    repository_root: Path,
) -> subprocess.CompletedProcess[str]:
    """Run the configured Ruff formatter check against one documentation example."""
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "ruff",
            "format",
            "--check",
            "--no-cache",
            "--stdin-filename",
            "documentation_example.py",
            "-",
        ],
        cwd=repository_root,
        input=source,
        capture_output=True,
        text=True,
        check=False,
    )


def test_python_examples_support_markdown_fence_variants(tmp_path: Path) -> None:
    """Recognize indentation, attributes, tildes, and longer closing fences."""
    markdown_path = tmp_path / "examples.md"
    markdown_path.write_text(
        '```python title="basic"\nfirst = 1\n```\n'
        "\n"
        '   ~~~python linenums="1"\n   second = 2\n   ~~~~\n'
        "\n"
        "```bash\necho ignored\n```\n"
        "\n"
        "````markdown\n```python\nnot_python_here = 1\n```\n````\n",
        encoding="utf-8",
    )

    assert _python_examples(markdown_path) == ["first = 1\n", "second = 2\n"]


def test_documentation_python_examples_pass_ruff() -> None:
    """Require every documented Python example to satisfy the Ruff policy."""
    repository_root = Path(__file__).resolve().parents[1]
    markdown_paths = [
        repository_root / "README.md",
        *sorted((repository_root / "docs").rglob("*.md")),
    ]
    failures: list[str] = []

    for path in markdown_paths:
        for example_number, example in enumerate(_python_examples(path), start=1):
            checks = (
                ("lint", _run_ruff_lint(example, repository_root)),
                ("format", _run_ruff_format(example, repository_root)),
            )
            for check_name, result in checks:
                if result.returncode != 0:
                    output = "\n".join(
                        part for part in (result.stdout, result.stderr) if part
                    )
                    failures.append(
                        f"{path.relative_to(repository_root)} example "
                        f"{example_number} {check_name}:\n{output}"
                    )

    assert not failures, "\n\n".join(failures)
