"""Tests for motor state data structures."""

from cubemars_servo_can.motor_state import ServoMotorState


def test_motor_state_str_contains_fields() -> None:
    state = ServoMotorState(
        position=1.0,
        velocity=2.0,
        current=3.0,
        temperature=4.0,
        error=5,
        acceleration=6.0,
    )
    text = str(state)
    assert "Position:" in text
    assert "Velocity:" in text
    assert "Error:" in text
