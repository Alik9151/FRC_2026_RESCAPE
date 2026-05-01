package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // indexer
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double velocityRPS = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets the motor's speed given an RPS input */
  public default void setVelocity(double rps) {}

  /** Stop the motor */
  public default void stop() {}
}
