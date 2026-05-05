package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  class OuttakeIOInputs {
    public boolean connected;
    public double appliedVolts;
    public double statorCurrentAmps;
    public double velocityRPS;

    public boolean isLoaded;
  }

  default void updateInputs(OuttakeIOInputs inputs) {}

  /** Sets the motor's speed given an RPS input */
  default void setVelocity(double rps) {}

  /** Stop the motor */
  default void stop() {}
}
