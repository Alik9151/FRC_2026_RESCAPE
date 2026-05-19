package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class OuttakeConstants {
  public static final double ROLLER_GEAR_RATIO = 1;
  public static final double ROLLER_MOI = 0.002;

  public static final double PIVOT_GEAR_RATIO = 1;
  public static final double PIVOT_MOI = 0.05;

  public static final double ROLLER_KP = 2.0;
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KS = 0;
  public static final double ROLLER_KV = 0.12;

  public static final double PIVOT_KP = 1.5;
  public static final double PIVOT_KI = 0;
  public static final double PIVOT_KD = 0;
  public static final double PIVOT_KS = 0;
  public static final double PIVOT_KV = 0.12;

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
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(PIVOT_GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  .withKP(PIVOT_KP)
                  .withKI(PIVOT_KI)
                  .withKD(PIVOT_KD)
                  .withKS(PIVOT_KS)
                  .withKV(PIVOT_KV));

  public static final double RPS = 80;
  public static final double STOWED_DEG = 0.0;
  public static final double DROPPING_DEG = 125.0;
  public static final double DROPPING_DEG_L4 = 90.0;
}
