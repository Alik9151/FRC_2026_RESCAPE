package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IntakeConstants {
  public static final double INTAKE_GEAR_RATIO = 1;
  public static final double INTAKE_MOI = 0.05;

  public static final double INTAKE_KP = 0.1;
  public static final double INTAKE_KI = 0;
  public static final double INTAKE_KD = 0;
  public static final double INTAKE_KS = 0;
  public static final double INTAKE_KV = 0.12;

  public static final TalonFXConfiguration INTAKE_CONFIG =
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
                  .withKP(INTAKE_KP)
                  .withKI(INTAKE_KI)
                  .withKD(INTAKE_KD)
                  .withKS(INTAKE_KS)
                  .withKV(INTAKE_KV));

  public static final double INTAKE_RPS = 5000 / 60.0;
}
