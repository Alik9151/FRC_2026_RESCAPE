package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class OuttakeConstants {
  public static final double OUTTAKE_ROLLER_GEAR_RATIO = 1;
  public static final double OUTTAKE_ROLLER_MOI = 0.002;

  public static final double OUTTAKE_PIVOT_GEAR_RATIO = 1;
  public static final double OUTTAKE_PIVOT_MOI = 0.05;

  public static final double OUTTAKE_ROLLER_KP = 2.0;
  public static final double OUTTAKE_ROLLER_KI = 0;
  public static final double OUTTAKE_ROLLER_KD = 0;
  public static final double OUTTAKE_ROLLER_KS = 0;
  public static final double OUTTAKE_ROLLER_KV = 0.12;

  public static final double OUTTAKE_PIVOT_KP = 2.0;
  public static final double OUTTAKE_PIVOT_KI = 0;
  public static final double OUTTAKE_PIVOT_KD = 0;
  public static final double OUTTAKE_PIVOT_KS = 0;
  public static final double OUTTAKE_PIVOT_KV = 0.12;

  public static final TalonFXConfiguration OUTTAKE_ROLLER_CONFIG =
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
                  .withKP(OUTTAKE_ROLLER_KP)
                  .withKI(OUTTAKE_ROLLER_KI)
                  .withKD(OUTTAKE_ROLLER_KD)
                  .withKS(OUTTAKE_ROLLER_KS)
                  .withKV(OUTTAKE_ROLLER_KV));

  public static final TalonFXConfiguration OUTTAKE_PIVOT_CONFIG =
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
                  .withKP(OUTTAKE_PIVOT_KP)
                  .withKI(OUTTAKE_PIVOT_KI)
                  .withKD(OUTTAKE_PIVOT_KD)
                  .withKS(OUTTAKE_PIVOT_KS)
                  .withKV(OUTTAKE_PIVOT_KV));

  public static final double OUTTAKE_RPS = 80;
  public static final double OUTTAKE_STORED_DEG = 0.0;
}
