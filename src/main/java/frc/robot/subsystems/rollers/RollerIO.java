package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  class RollerIOInputs {
    public boolean connected;
    public double velocityRPS;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double statorCurrentAmps;
    public double tempCelsius;

    public boolean[] followerConnected;
    public double[] followerTempCelsius;
  }

  enum RollerIOMode {
    COAST,
    BRAKE,
    VOLTAGE_CONTROL,
    CLOSED_LOOP
  }

  default void updateInputs(RollerIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setVelocity(double rps) {}

  default void coast() {}

  default void brake() {}

  default int getNumFollowers() {
    return 0;
  }
}
