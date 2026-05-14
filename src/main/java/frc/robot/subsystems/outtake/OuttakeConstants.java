package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class OuttakeConstants {
  public static final double OUTTAKE_GEAR_RATIO = 1.0;
  public static final double OUTTAKE_MOI = 0.001;

  public static final double OUTTAKE_KP = 0.1;
  public static final double OUTTAKE_KI = 0;
  public static final double OUTTAKE_KD = 0;
  public static final double OUTTAKE_KS = 0;
  public static final double OUTTAKE_KV = 0.12;

  public static final TalonFXConfiguration OUTTAKE_CONFIG =
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
                  .withNeutralMode(NeutralModeValue.Brake))
          .withSlot0(
              new Slot0Configs()
                  .withKP(OUTTAKE_KP)
                  .withKI(OUTTAKE_KI)
                  .withKD(OUTTAKE_KD)
                  .withKS(OUTTAKE_KS)
                  .withKV(OUTTAKE_KV));

  public static final double OUTTAKE_RPS = 60;
}
