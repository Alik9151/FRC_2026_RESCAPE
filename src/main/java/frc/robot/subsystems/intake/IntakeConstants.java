package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IntakeConstants {
  public static final double INTAKE_ROLLER_GEAR_RATIO = 1;
  public static final double INTAKE_ROLLER_MOI = 0.002;

  public static final double INTAKE_PIVOT_GEAR_RATIO = 1;
  public static final double INTAKE_PIVOT_MOI = 0.05;

  public static final double INTAKE_ROLLER_KP = 2.0;
  public static final double INTAKE_ROLLER_KI = 0;
  public static final double INTAKE_ROLLER_KD = 0;
  public static final double INTAKE_ROLLER_KS = 0;
  public static final double INTAKE_ROLLER_KV = 0.12;

  public static final double INTAKE_PIVOT_KP = 2.0;
  public static final double INTAKE_PIVOT_KI = 0;
  public static final double INTAKE_PIVOT_KD = 0;
  public static final double INTAKE_PIVOT_KS = 0;
  public static final double INTAKE_PIVOT_KV = 0.12;

  public static final TalonFXConfiguration INTAKE_ROLLER_CONFIG =
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
                  .withKP(INTAKE_ROLLER_KP)
                  .withKI(INTAKE_ROLLER_KI)
                  .withKD(INTAKE_ROLLER_KD)
                  .withKS(INTAKE_ROLLER_KS)
                  .withKV(INTAKE_ROLLER_KV));

  public static final TalonFXConfiguration INTAKE_PIVOT_CONFIG =
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
                  .withKP(INTAKE_PIVOT_KP)
                  .withKI(INTAKE_PIVOT_KI)
                  .withKD(INTAKE_PIVOT_KD)
                  .withKS(INTAKE_PIVOT_KS)
                  .withKV(INTAKE_PIVOT_KV));

  public static final double INTAKE_RPS = 80;
  public static final double INTAKE_POSITION_DEG = 110;
}
