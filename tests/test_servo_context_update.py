import pytest
from typing import Dict, Any
from unittest.mock import patch

from cubemars_servo_can.constants import ControlMode
from cubemars_servo_can.servo_can import CubeMarsServoCAN


class TestContextManagerAndUpdateBranches:
    """Tests for context manager, CSV logging, and update error paths."""

    def test_enter_with_csv_creates_writer(
        self, mock_can: Dict[str, Any], tmp_path
    ) -> None:
        csv_path = tmp_path / "motor_log.csv"
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, CSV_file=str(csv_path)
        )

        with patch.object(motor, "check_can_connection", return_value=True):
            with motor:
                assert motor.csv_file is not None

        content = csv_path.read_text().strip().splitlines()
        assert content[0].startswith("pi_time,")

    def test_enter_raises_if_connection_check_fails(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        with patch.object(motor, "check_can_connection", return_value=False):
            with pytest.raises(RuntimeError, match="Device not connected"):
                motor.__enter__()
        assert motor._entered is False

    def test_enter_failure_after_power_on_still_powers_off(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)

        with patch.object(
            motor, "_send_command", side_effect=RuntimeError("send boom")
        ):
            with patch.object(motor, "power_off") as power_off:
                with pytest.raises(RuntimeError, match="send boom"):
                    motor.__enter__()
                power_off.assert_called_once()

        assert motor._entered is False

    def test_enter_failure_closes_csv_and_swallows_power_off_failure(
        self, mock_can: Dict[str, Any], tmp_path
    ) -> None:
        csv_path = tmp_path / "motor_log.csv"
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, CSV_file=str(csv_path)
        )
        with patch.object(motor, "check_can_connection", return_value=False):
            with patch.object(
                motor, "power_off", side_effect=RuntimeError("off failed")
            ):
                with pytest.raises(RuntimeError, match="Device not connected"):
                    motor.__enter__()

        assert motor._entered is False
        assert motor.csv_file is None

    def test_exit_closes_csv_and_prints_traceback(
        self, mock_can: Dict[str, Any], tmp_path
    ) -> None:
        csv_path = tmp_path / "motor_log.csv"
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, CSV_file=str(csv_path)
        )
        with patch.object(motor, "check_can_connection", return_value=True):
            motor.__enter__()

        err = RuntimeError("boom")
        with patch("cubemars_servo_can.servo_can.traceback.print_exception") as pe:
            motor.__exit__(RuntimeError, err, None)
            pe.assert_called_once()
        assert motor.csv_file is None
        assert motor._entered is False

    def test_exit_soft_stops_velocity_before_zero_current_shutdown(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()
        motor._command.velocity = 1000.0

        with patch.object(motor, "_send_command") as send_command:
            with patch.object(motor._canman, "comm_can_set_current") as set_current:
                with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                    motor.__exit__(None, None, None)

        assert send_command.call_count == motor.soft_stop_ramp_steps
        assert sleep.call_count == motor.soft_stop_ramp_steps
        set_current.assert_called_once_with(motor.ID, 0.0)
        assert motor._command.velocity == 0.0
        assert motor._entered is False

    def test_exit_soft_stops_position_before_zero_current_shutdown(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_control()
        motor._motor_state_async.position = 100.0
        motor._last_update_time = motor._start_time + 1.0
        expected_hold = (
            motor._motor_state_async.position
            * motor.rad_per_Eang
            / motor.config.GEAR_RATIO
        )

        with patch.object(motor, "set_output_angle_radians") as set_pos:
            with patch.object(motor, "_send_command") as send_command:
                with patch.object(motor._canman, "comm_can_set_current") as set_current:
                    with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                        motor.__exit__(None, None, None)

        set_pos.assert_called_once_with(expected_hold)
        assert send_command.call_count == motor.soft_stop_ramp_steps
        assert sleep.call_count == motor.soft_stop_ramp_steps
        set_current.assert_called_once_with(motor.ID, 0.0)
        assert motor._entered is False

    def test_exit_soft_stops_position_velocity_before_zero_current_shutdown(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_velocity_control()
        motor._motor_state_async.position = 150.0
        motor._last_update_time = motor._start_time + 1.0
        expected_hold = (
            motor._motor_state_async.position
            * motor.rad_per_Eang
            / motor.config.GEAR_RATIO
        )

        with patch.object(motor, "set_output_angle_radians") as set_pos:
            with patch.object(motor, "_send_command") as send_command:
                with patch.object(motor._canman, "comm_can_set_current") as set_current:
                    with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                        motor.__exit__(None, None, None)

        set_pos.assert_called_once_with(expected_hold, 0.0, 0.0)
        assert send_command.call_count == motor.soft_stop_ramp_steps
        assert sleep.call_count == motor.soft_stop_ramp_steps
        set_current.assert_called_once_with(motor.ID, 0.0)
        assert motor._entered is False

    def test_exit_soft_stop_position_falls_back_to_command_when_no_telemetry(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_control()
        motor._command.position = 12.5
        motor._last_update_time = motor._start_time
        expected_hold = motor._command.position * motor.rad_per_Eang

        with patch.object(motor, "set_output_angle_radians") as set_pos:
            with patch.object(motor, "_send_command"):
                with patch.object(motor._canman, "comm_can_set_current"):
                    with patch("cubemars_servo_can.servo_can.time.sleep"):
                        motor.__exit__(None, None, None)

        set_pos.assert_called_once_with(expected_hold)

    def test_exit_soft_stop_position_velocity_falls_back_to_command_when_no_telemetry(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_velocity_control()
        motor._command.position = 7.5
        motor._last_update_time = motor._start_time
        expected_hold = motor._command.position * motor.rad_per_Eang

        with patch.object(motor, "set_output_angle_radians") as set_pos:
            with patch.object(motor, "_send_command"):
                with patch.object(motor._canman, "comm_can_set_current"):
                    with patch("cubemars_servo_can.servo_can.time.sleep"):
                        motor.__exit__(None, None, None)

        set_pos.assert_called_once_with(expected_hold, 0.0, 0.0)

    def test_soft_stop_returns_early_when_not_entered(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = False
        motor.enter_velocity_control()
        motor._command.velocity = 1000.0

        with patch.object(motor, "_send_command") as send_command:
            motor._best_effort_soft_stop()

        send_command.assert_not_called()
        assert motor._command.velocity == 1000.0

    def test_soft_stop_duty_cycle_ramps_to_zero(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_duty_cycle_control()
        motor._command.duty = 0.6

        with patch.object(motor, "_send_command") as send_command:
            with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                motor._best_effort_soft_stop()

        assert motor._command.duty == 0.0
        assert send_command.call_count == motor.soft_stop_ramp_steps
        assert sleep.call_count == motor.soft_stop_ramp_steps

    @pytest.mark.parametrize(
        "mode_setter",
        ["enter_current_control", "enter_current_brake_control"],
    )
    def test_soft_stop_current_modes_ramp_to_zero(
        self, mock_can: Dict[str, Any], mode_setter: str
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        getattr(motor, mode_setter)()
        motor._command.current = 5.0

        with patch.object(motor, "_send_command") as send_command:
            with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                motor._best_effort_soft_stop()

        assert motor._command.current == 0.0
        assert send_command.call_count == motor.soft_stop_ramp_steps
        assert sleep.call_count == motor.soft_stop_ramp_steps

    def test_soft_stop_position_swallows_setpoint_exception_and_continues(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_control()
        motor._motor_state_async.position = 5.0
        motor._last_update_time = motor._start_time + 1.0

        with patch.object(
            motor, "set_output_angle_radians", side_effect=RuntimeError("boom")
        ):
            with patch.object(motor, "_send_command") as send_command:
                with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                    motor._best_effort_soft_stop()

        assert send_command.call_count == motor.soft_stop_ramp_steps
        assert sleep.call_count == motor.soft_stop_ramp_steps

    def test_soft_stop_position_velocity_swallows_setpoint_exception_and_continues(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_velocity_control()
        motor._motor_state_async.position = 5.0
        motor._last_update_time = motor._start_time + 1.0

        with patch.object(
            motor, "set_output_angle_radians", side_effect=RuntimeError("boom")
        ):
            with patch.object(motor, "_send_command") as send_command:
                with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                    motor._best_effort_soft_stop()

        assert send_command.call_count == motor.soft_stop_ramp_steps
        assert sleep.call_count == motor.soft_stop_ramp_steps

    def test_soft_stop_optional_brake_hold_sequence(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9",
            motor_ID=1,
            soft_stop_ramp_duration_s=0.04,
            soft_stop_ramp_steps=4,
            soft_stop_brake_hold_current_amps=2.0,
            soft_stop_brake_hold_duration_s=0.03,
        )
        motor._entered = True
        motor.enter_velocity_control()
        motor._command.velocity = 1200.0

        snapshots = []

        def capture_command() -> None:
            snapshots.append(
                (motor._control_state, motor._command.velocity, motor._command.current)
            )

        with patch.object(motor, "_send_command", side_effect=capture_command) as send:
            with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                motor._best_effort_soft_stop()

        assert send.call_count == 8
        assert sleep.call_count == 8
        assert snapshots[0][0] == ControlMode.VELOCITY
        assert snapshots[3][1] == 0.0
        assert snapshots[4][0] == ControlMode.CURRENT_BRAKE
        assert snapshots[4][2] == pytest.approx(2.0)
        assert snapshots[-1][0] == ControlMode.CURRENT_BRAKE
        assert snapshots[-1][2] == 0.0
        assert motor._control_state == ControlMode.VELOCITY

    def test_optional_brake_hold_noop_when_config_current_limit_is_zero(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9",
            motor_ID=1,
            soft_stop_brake_hold_current_amps=2.0,
            soft_stop_brake_hold_duration_s=0.05,
            config_overrides={"Curr_max": 0.0},
        )
        motor._entered = True
        motor.enter_velocity_control()
        motor._command.velocity = 1000.0

        with patch.object(motor, "_send_command") as send_command:
            with patch("cubemars_servo_can.servo_can.time.sleep"):
                motor._best_effort_soft_stop()

        assert send_command.call_count == motor.soft_stop_ramp_steps

    def test_soft_stop_swallows_outer_send_exception(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_duty_cycle_control()
        motor._command.duty = 0.3

        with patch.object(
            motor, "_send_command", side_effect=RuntimeError("send boom")
        ):
            # Should not raise
            motor._best_effort_soft_stop()

        assert motor._command.duty == 0.0

    def test_soft_stop_swallows_outer_send_exception_velocity_mode_sets_zero(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_velocity_control()
        motor._command.velocity = 123.0

        with patch.object(motor, "_send_command", side_effect=RuntimeError("boom")):
            motor._best_effort_soft_stop()

        assert motor._command.velocity == 0.0

    def test_soft_stop_swallows_outer_send_exception_current_mode_sets_zero(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_current_control()
        motor._command.current = 6.0

        with patch.object(motor, "_send_command", side_effect=RuntimeError("boom")):
            motor._best_effort_soft_stop()

        assert motor._command.current == 0.0

    def test_soft_stop_swallows_outer_send_exception_position_velocity_sets_zero_profile(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor.enter_position_velocity_control()
        motor._command.velocity = 900.0
        motor._command.acceleration = 700.0

        with patch.object(motor, "_send_command", side_effect=RuntimeError("boom")):
            motor._best_effort_soft_stop()

        assert motor._command.velocity == 0.0
        assert motor._command.acceleration == 0.0

    def test_soft_stop_default_case_is_noop(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor._control_state = "UNKNOWN_MODE"  # type: ignore[assignment]

        with patch.object(motor, "_send_command") as send_command:
            with patch("cubemars_servo_can.servo_can.time.sleep") as sleep:
                motor._best_effort_soft_stop()

        send_command.assert_not_called()
        sleep.assert_not_called()

    def test_update_raises_when_not_entered(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        with pytest.raises(RuntimeError, match="before safely powering on"):
            motor.update()

    def test_update_raises_when_temperature_exceeds_limit(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, overtemp_trip_count=1
        )
        motor._entered = True
        motor._motor_state_async.temperature = motor.max_temp + 1.0
        with pytest.raises(RuntimeError, match="Temperature greater than"):
            motor.update()

    def test_update_ignores_single_overtemp_spike_with_trip_count(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, overtemp_trip_count=3
        )
        motor._entered = True

        motor._motor_state_async.temperature = motor.max_temp + 1.0
        motor.update()
        assert motor._overtemp_samples == 1

        motor._motor_state_async.temperature = motor.max_temp - 1.0
        motor.update()
        assert motor._overtemp_samples == 0

    def test_update_pretrip_thermal_guard_suppresses_velocity_motion(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, overtemp_trip_count=3
        )
        motor._entered = True
        motor.enter_velocity_control()
        motor._command.velocity = 4321.0
        motor._motor_state_async.temperature = motor.max_temp + 5.0

        with patch.object(motor._canman, "comm_can_set_rpm") as set_rpm:
            motor.update()

        set_rpm.assert_called_once_with(motor.ID, 0.0)
        assert motor._command.velocity == 4321.0
        assert motor._thermal_guard_active is True
        assert motor._overtemp_samples == 1

    def test_update_pretrip_thermal_guard_holds_position_velocity_mode(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, overtemp_trip_count=3
        )
        motor._entered = True
        motor.enter_position_velocity_control()
        motor._command.position = 99.0
        motor._command.velocity = 1000.0
        motor._command.acceleration = 2000.0
        motor._motor_state_async.position = 55.0
        motor._last_update_time = motor._start_time + 1.0
        motor._motor_state_async.temperature = motor.max_temp + 2.0

        with patch.object(motor._canman, "comm_can_set_pos_spd") as set_pos_spd:
            motor.update()

        set_pos_spd.assert_called_once_with(motor.ID, 55.0, 0.0, 0.0)
        assert motor._command.position == 99.0
        assert motor._command.velocity == 1000.0
        assert motor._command.acceleration == 2000.0

    def test_thermal_guard_requires_hysteresis_cooldown_to_clear(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9",
            motor_ID=1,
            overtemp_trip_count=3,
            cooldown_margin_c=2.0,
        )
        motor._entered = True
        motor.enter_velocity_control()
        motor._command.velocity = 2500.0

        with patch.object(motor._canman, "comm_can_set_rpm") as set_rpm:
            motor._motor_state_async.temperature = motor.max_temp + 1.0
            motor.update()

            motor._motor_state_async.temperature = motor.max_temp - 1.0
            motor.update()

            motor._motor_state_async.temperature = motor.max_temp - 3.0
            motor.update()

        assert [call.args[1] for call in set_rpm.call_args_list] == [0.0, 0.0, 2500.0]
        assert motor._thermal_guard_active is False

    @pytest.mark.parametrize(
        "mode",
        [
            ControlMode.DUTY_CYCLE,
            ControlMode.CURRENT_LOOP,
            ControlMode.CURRENT_BRAKE,
            ControlMode.VELOCITY,
            ControlMode.POSITION,
            ControlMode.POSITION_VELOCITY,
            ControlMode.IDLE,
        ],
    )
    def test_send_thermal_guard_command_restores_user_command(
        self, mock_can: Dict[str, Any], mode: ControlMode
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor._control_state = mode
        motor._command.position = 12.0
        motor._command.velocity = 34.0
        motor._command.current = 5.0
        motor._command.duty = 0.4
        motor._command.acceleration = 56.0
        motor._motor_state_async.position = 78.0
        motor._last_update_time = motor._start_time + 1.0

        with patch.object(motor, "_send_command") as send_command:
            motor._send_thermal_guard_command()

        send_command.assert_called_once()
        assert motor._command.position == 12.0
        assert motor._command.velocity == 34.0
        assert motor._command.current == 5.0
        assert motor._command.duty == 0.4
        assert motor._command.acceleration == 56.0

    def test_update_raises_after_consecutive_overtemp_samples(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, overtemp_trip_count=2
        )
        motor._entered = True
        motor._motor_state_async.temperature = motor.max_temp + 1.0
        motor.update()
        with pytest.raises(RuntimeError, match="Temperature greater than"):
            motor.update()

    def test_update_does_not_latch_stale_overtemp_when_async_state_cooled(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor._motor_state.temperature = motor.max_temp + 10.0
        motor._motor_state_async.temperature = motor.max_temp - 5.0

        motor.update()

        assert motor.temperature == pytest.approx(motor.max_temp - 5.0)

    def test_update_warns_on_stale_state(self, mock_can: Dict[str, Any]) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True
        motor._last_command_time = 9.9
        motor._last_update_time = 9.0
        with patch("time.time", return_value=10.0):
            with pytest.warns(
                RuntimeWarning, match="State update requested but no data"
            ):
                motor.update()

    def test_update_writes_csv_row(self, mock_can: Dict[str, Any], tmp_path) -> None:
        csv_path = tmp_path / "motor_log.csv"
        motor: CubeMarsServoCAN = CubeMarsServoCAN(
            motor_type="AK80-9", motor_ID=1, CSV_file=str(csv_path)
        )
        with patch.object(motor, "check_can_connection", return_value=True):
            motor.__enter__()

        motor._motor_state_async.position = 1.0
        motor._motor_state_async.velocity = 2.0
        motor._motor_state_async.current = 3.0
        motor._motor_state_async.temperature = 4.0
        motor.update()
        motor.__exit__(None, None, None)

        lines = csv_path.read_text().strip().splitlines()
        assert len(lines) >= 2

    def test_exit_warns_if_zero_current_shutdown_fails(
        self, mock_can: Dict[str, Any]
    ) -> None:
        motor: CubeMarsServoCAN = CubeMarsServoCAN(motor_type="AK80-9", motor_ID=1)
        motor._entered = True

        with patch.object(motor, "_best_effort_soft_stop"):
            with patch.object(
                motor._canman, "comm_can_set_current", side_effect=RuntimeError("boom")
            ):
                with pytest.warns(
                    RuntimeWarning, match="Zero-current shutdown command failed"
                ):
                    motor.__exit__(None, None, None)

        assert motor._entered is False
