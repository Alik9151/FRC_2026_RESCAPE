package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean connected;
    public double appliedVolts;
    public double statorCurrentAmps;
    public double velocityRPS;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets the motor's speed given an RPS input */
  default void setVelocity(double rps) {}

  /** Stop the motor */
  default void stop() {}
}
