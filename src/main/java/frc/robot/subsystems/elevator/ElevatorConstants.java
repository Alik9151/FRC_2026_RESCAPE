package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.Elevator.ElevatorState.*;
import static frc.robot.subsystems.elevator.Elevator.distanceToAngle;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import java.util.HashMap;
import java.util.Map;

public final class ElevatorConstants {
  public static final int STATOR_CURRENT_LIMIT = 120;
  public static final int SUPPLY_CURRENT_LIMIT = 60;
  public static final InvertedValue ELEVATOR_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue ELEVATOR_NEUTRAL_OUTPUT = NeutralModeValue.Coast;

  public static final double ELEVATOR_GEAR_RATIO = 75;

  public static final TalonFXConfiguration ELEVATOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                  .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(ELEVATOR_INVERTED)
                  .withNeutralMode(ELEVATOR_NEUTRAL_OUTPUT));

  static {
    ELEVATOR_CONFIG.Slot0.kP = 0.1;
    ELEVATOR_CONFIG.Slot0.kI = 0;
    ELEVATOR_CONFIG.Slot0.kD = 0;
    ELEVATOR_CONFIG.Slot0.kS = 0;
    ELEVATOR_CONFIG.Slot0.kV = 0;
    ELEVATOR_CONFIG.Slot0.kA = 0;
    ELEVATOR_CONFIG.Slot0.kG = 0;
    ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 5;
    ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 5;

    ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_RATIO;
  }

  public static final double MAX_MANUAL_VOLTAGE = 6.0;
  public static final double HOMING_VOLTAGE = 2.0;
  public static final double HOMING_VELOCITY_THRESHOLD = 5.0;

  public static final Map<Elevator.ElevatorState, Angle> SETPOINTS = new HashMap<>();

  static {
    SETPOINTS.put(STOWED, distanceToAngle(0.0));
  }
}
