package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;

public final class IntakeConstants {
  public static final int INDEXER_STATOR_LIMIT = 60;
  public static final int INDEXER_SUPPLY_LIMIT = 40;

  public static final InvertedValue INDEXER_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final double INDEXER_MOI = 0.001;

  public static final double INDEXER_KP = 0.1;
  public static final double INDEXER_KI = 0;
  public static final double INDEXER_KD = 0;
  public static final double INDEXER_KS = 0;
  public static final double INDEXER_KV = 0.12;

  public static final double INTAKE_RPS = 5000 / 60.0;
}
