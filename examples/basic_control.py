"""Minimal velocity-control example for one CubeMars actuator."""

import time

from cubemars_servo_can import (
    ControlMode,
    CubeMarsServoCan,
    MotorModel,
    ServoConfig,
)


def main() -> None:
    """Run a short, low-speed example after hardware commissioning."""
    config = ServoConfig(
        motor=MotorModel.AK80_9,
        motor_id=1,
        max_current_amps=5.0,
        max_output_torque_newton_meters=4.0,
    )
    with CubeMarsServoCan(config) as motor:
        motor.set_control_mode(ControlMode.VELOCITY)
        motor.set_output_velocity(1.0)
        for _ in range(100):
            motor.update()
            time.sleep(0.01)


if __name__ == "__main__":
    main()
