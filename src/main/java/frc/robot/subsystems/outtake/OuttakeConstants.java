package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class OuttakeConstants {
  public static final double MOTOR_TO_SENSOR = 1;
  public static final double SENSOR_TO_PIVOT = 1; // for running absolute encoder before reduction
  public static final double PIVOT_GEAR_RATIO = MOTOR_TO_SENSOR * SENSOR_TO_PIVOT;
  public static final double PIVOT_MOI = 0.05;

  public static final double ROLLER_GEAR_RATIO = 1;
  public static final double ROLLER_MOI = 0.002;

  public static final double PIVOT_KP = 1.5;
  public static final double PIVOT_KI = 0;
  public static final double PIVOT_KD = 0;
  public static final double PIVOT_KS = 0;
  public static final double PIVOT_KV = 0.12;

  public static final double ROLLER_KP = 2.0;
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KS = 0;
  public static final double ROLLER_KV = 0.12;

  public static final TalonFXConfiguration PIVOT_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(
              new FeedbackConfigs()
                  .withRotorToSensorRatio(MOTOR_TO_SENSOR)
                  .withSensorToMechanismRatio(SENSOR_TO_PIVOT))
          .withSlot0(
              new Slot0Configs()
                  .withKP(PIVOT_KP)
                  .withKI(PIVOT_KI)
                  .withKD(PIVOT_KD)
                  .withKS(PIVOT_KS)
                  .withKV(PIVOT_KV));

  public static final TalonFXConfiguration ROLLER_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(
              new Slot0Configs()
                  .withKP(ROLLER_KP)
                  .withKI(ROLLER_KI)
                  .withKD(ROLLER_KD)
                  .withKS(ROLLER_KS)
                  .withKV(ROLLER_KV));

  public static final CANcoderConfiguration ENCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withMagnetOffset(0)
                  .withSensorDirection(
                      PIVOT_CONFIG.MotorOutput.Inverted == InvertedValue.CounterClockwise_Positive
                          ? SensorDirectionValue.CounterClockwise_Positive
                          : SensorDirectionValue.Clockwise_Positive));

  public static final double STOWED_DEG = 0.0;
  public static final double DROPPING_DEG = 125.0;
  public static final double DROPPING_DEG_L4 = 90.0;
  public static final double RPS = 80;
}
