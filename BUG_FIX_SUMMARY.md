# Verified 1.0 bug register

The tests are the source of truth for these corrections.

| ID | Correction | Regression coverage |
| --- | --- | --- |
| BUG-035 | Profile speed/acceleration encoded without the documented divide-by-10 scale | Golden `SET_POS_SPD` frame |
| BUG-036 | Status temperature decoded as unsigned | Negative-temperature decode vector |
| BUG-037 | Arbitrary high-byte and exact motor-ID traffic accepted as status | Exact extended `0x29` routing tests |
| BUG-038 | Fault 6 used an obsolete description and fault 7 was absent | Typed fault mapping tests |
| BUG-039 | Servo lifecycle emitted MIT FC/FD frames | Probe and shutdown frame tests |
| BUG-040 | One global CAN singleton prevented independent channels | Multi-channel transport tests |
| BUG-041 | Duplicate IDs could consume the same status stream | Duplicate registration rollback test |
| BUG-042 | AKA60-6 software current cap used a 60 A controller limit | Configuration safety tests |
| BUG-043 | AK80-9 exposed torque targets unreachable under its current cap | Configuration coherence tests |
| BUG-044 | Listener failures and motor faults lacked specific public exceptions | User-thread exception tests |

## Validation

The required release commands are:

```bash
uv run ruff format --check .
uv run ruff check .
uv run mypy
uv run vulture
uv run pytest
```

The suite also verifies byte-level protocol vectors, lifecycle rollback, transport
ownership, thermal behavior, CSV output, immutable dataclass policy, dead-code
policy, and all documented Python examples.
