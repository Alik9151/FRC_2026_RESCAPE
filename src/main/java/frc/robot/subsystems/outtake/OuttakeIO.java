package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public static class OuttakeIOInputs {
    // Outtake
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double velocityRPS = 0.0;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  /** Sets the motor's speed given an RPS input */
  public default void setVelocity(double rps) {}

  /** Stop the motor */
  public default void stop() {}
}
