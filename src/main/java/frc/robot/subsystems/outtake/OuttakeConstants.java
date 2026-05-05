package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.signals.InvertedValue;

public final class OuttakeConstants {
  public static final int OUTTAKE_STATOR_LIMIT = 60;
  public static final int OUTTAKE_SUPPLY_LIMIT = 40;

  public static final InvertedValue OUTTAKE_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final double OUTTAKE_MOI = 0.001;
  public static final double OUTTAKE_GEAR_RATIO = 1.0;

  public static final double OUTTAKE_KP = 0.1;
  public static final double OUTTAKE_KI = 0;
  public static final double OUTTAKE_KD = 0;
  public static final double OUTTAKE_KS = 0;
  public static final double OUTTAKE_KV = 0.12;

  public static final double OUTTAKE_RPS = 5000 / 60.0;
}
