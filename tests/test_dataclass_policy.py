"""Repository-wide dataclass options and ordering enforcement tests."""

from __future__ import annotations

import ast
from pathlib import Path

_DATACLASS_OPTION_ORDER = (
    "init",
    "repr",
    "eq",
    "order",
    "unsafe_hash",
    "frozen",
    "match_args",
    "kw_only",
    "slots",
    "weakref_slot",
)
_DATACLASS_OPTION_POSITION = {
    option: position for position, option in enumerate(_DATACLASS_OPTION_ORDER)
}
_REQUIRED_OPTIONS = ("kw_only", "slots")


def _is_literal_true(expression: ast.expr | None) -> bool:
    """Return whether an AST expression is the literal value true."""
    return isinstance(expression, ast.Constant) and expression.value is True


def _dataclass_bindings(tree: ast.Module) -> tuple[set[str], set[str]]:
    """Return direct decorator names and imported dataclasses module names."""
    decorator_names: set[str] = set()
    module_names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "dataclasses":
            decorator_names.update(
                alias.asname or alias.name
                for alias in node.names
                if alias.name == "dataclass"
            )
        elif isinstance(node, ast.Import):
            module_names.update(
                alias.asname or alias.name
                for alias in node.names
                if alias.name == "dataclasses"
            )
    return decorator_names, module_names


def _is_dataclass_reference(
    expression: ast.expr,
    decorator_names: set[str],
    module_names: set[str],
) -> bool:
    """Recognize imported direct and module-qualified dataclass references."""
    if isinstance(expression, ast.Name):
        return expression.id in decorator_names
    return (
        isinstance(expression, ast.Attribute)
        and expression.attr == "dataclass"
        and isinstance(expression.value, ast.Name)
        and expression.value.id in module_names
    )


def _dataclass_keywords(
    decorator: ast.expr,
    decorator_names: set[str],
    module_names: set[str],
) -> list[ast.keyword] | None:
    """Return explicit keywords when `decorator` is an imported dataclass."""
    reference = decorator.func if isinstance(decorator, ast.Call) else decorator
    if not _is_dataclass_reference(reference, decorator_names, module_names):
        return None
    if not isinstance(decorator, ast.Call):
        return []
    return decorator.keywords


def _violations(
    path: Path,
    repository_root: Path,
) -> list[str]:
    """Return invalid dataclass declarations from one Python file."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    decorator_names, module_names = _dataclass_bindings(tree)
    relative_path = path.relative_to(repository_root)
    violations: list[str] = []

    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        for decorator in node.decorator_list:
            keywords = _dataclass_keywords(
                decorator,
                decorator_names,
                module_names,
            )
            if keywords is None:
                continue
            options = {
                keyword.arg: keyword.value
                for keyword in keywords
                if keyword.arg is not None
            }
            missing = [
                option
                for option in _REQUIRED_OPTIONS
                if not _is_literal_true(options.get(option))
            ]
            if missing:
                violations.append(
                    f"{relative_path}:{node.lineno}: {node.name} must set "
                    + ", ".join(f"{option}=True" for option in missing)
                )
            option_names = [
                keyword.arg for keyword in keywords if keyword.arg is not None
            ]
            canonical_option_names = sorted(
                option_names,
                key=lambda option: _DATACLASS_OPTION_POSITION.get(
                    option,
                    len(_DATACLASS_OPTION_POSITION),
                ),
            )
            if option_names != canonical_option_names:
                violations.append(
                    f"{relative_path}:{node.lineno}: {node.name} must order "
                    "dataclass options as " + ", ".join(canonical_option_names)
                )

    return violations


def test_dataclasses_follow_policy() -> None:
    """Require consistent dataclass construction and option ordering."""
    repository_root = Path(__file__).resolve().parents[1]
    violations = [
        violation
        for directory in ("src", "tests")
        for path in sorted((repository_root / directory).rglob("*.py"))
        for violation in _violations(path, repository_root)
    ]

    assert not violations, "Dataclass policy violations:\n" + "\n".join(violations)
