package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.elevator.Elevator.ElevatorState.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import java.util.EnumMap;
import java.util.Map;

public final class ElevatorConstants {
  public static double radiansToMeters(double radians) {
    return radians * DRUM_RADIUS;
  }

  public static double metersToRadians(double meters) {
    return meters / DRUM_RADIUS;
  }

  // physical constants
  public static final double ELEVATOR_GEAR_RATIO = 75;
  public static final double CARRIAGE_MASS = 15;
  public static final double DRUM_RADIUS = 0.2;
  public static final double MAX_HEIGHT = 4;
  public static final double STAGE_1_MAX_HEIGHT = 1.5;

  public static final TalonFXConfiguration ELEVATOR_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(120)
                  .withSupplyCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO)
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
  // closed loop
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
  }

  public static final double MAX_MANUAL_VOLTAGE = 6.0;
  public static final double HOMING_VOLTAGE = 2.0;
  public static final double HOMING_VELOCITY_THRESHOLD = 5.0;

  public static final Map<Elevator.ElevatorState, Angle> SETPOINTS =
      new EnumMap<>(Elevator.ElevatorState.class);

  static {
    SETPOINTS.put(STOWED, Radians.of(metersToRadians(0)));
    SETPOINTS.put(CORAL_L1, Radians.of(metersToRadians(0.2)));
    SETPOINTS.put(CORAL_L2, Radians.of(metersToRadians(0.5)));
    SETPOINTS.put(CORAL_L3, Radians.of(metersToRadians(.9)));
    SETPOINTS.put(CORAL_L4, Radians.of(metersToRadians(1.35)));
  }
}
