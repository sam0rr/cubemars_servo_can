class ServoMotorState:
    """
    Data structure to store and update motor states.
    Stores the telemetry data received from the motor.
    Values are kept in raw servo telemetry units and converted by higher-level getters.
    """

    def __init__(
        self,
        position: float,
        velocity: float,
        current: float,
        temperature: float,
        error: int,
        acceleration: float,
    ) -> None:
        """
        Sets the motor state to the input.

        Args:
            position: Position in electrical degrees
            velocity: Velocity in ERPM
            current: Current in amps
            temperature: Temperature in degrees C
            error: Error code, 0 means no error
            acceleration: Acceleration in ERPM/s
        """
        self.set_state(position, velocity, current, temperature, error, acceleration)

    def set_state(
        self,
        position: float,
        velocity: float,
        current: float,
        temperature: float,
        error: int,
        acceleration: float,
    ) -> None:
        """
        Sets the motor state to the input.

        Args:
            position: Position in electrical degrees
            velocity: Velocity in ERPM
            current: Current in amps
            temperature: Temperature in degrees C
            error: Error code, 0 means no error
            acceleration: Acceleration in ERPM/s
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_motor_state: "ServoMotorState") -> None:
        """
        Sets this motor state object's values to those of another motor state object.

        Args:
            other_motor_state: The other motor state object with values to set this motor state object's values to.
        """
        self.position = other_motor_state.position
        self.velocity = other_motor_state.velocity
        self.current = other_motor_state.current
        self.temperature = other_motor_state.temperature
        self.error = other_motor_state.error
        self.acceleration = other_motor_state.acceleration

    def __str__(self) -> str:
        """
        Returns a string representation of the motor state.
        """
        return f"Position: {self.position} | Velocity: {self.velocity} | Current: {self.current} | Temperature: {self.temperature} | Error: {self.error}"


class ServoCommand:
    """
    Data structure to store Servo command that will be sent upon update.
    Stores the desired target values for the motor.
    """

    def __init__(
        self,
        position: float,
        velocity: float,
        current: float,
        duty: float,
        acceleration: float,
    ) -> None:
        """
        Sets the motor command to the input.

        Args:
            position: Position in deg
            velocity: Velocity in ERPM
            current: Current in amps
            duty: Duty cycle in percentage ratio (-1 to 1)
            acceleration: Acceleration in ERPM/s
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.duty = duty
        self.acceleration = acceleration
