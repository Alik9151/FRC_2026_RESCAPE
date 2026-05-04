package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;

public final class IntakeConstants {
  public static final int INTAKE_STATOR_LIMIT = 60;
  public static final int INTAKE_SUPPLY_LIMIT = 40;

  public static final InvertedValue INTAKE_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final double INTAKE_MOI = 0.001;
  public static final double INTAKE_GEAR_RATIO = 1;

  public static final double INTAKE_KP = 0.1;
  public static final double INTAKE_KI = 0;
  public static final double INTAKE_KD = 0;
  public static final double INTAKE_KS = 0;
  public static final double INTAKE_KV = 0.12;

  public static final double INTAKE_RPS = 5000 / 60.0;
}
