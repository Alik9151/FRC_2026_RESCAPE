package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderConnected;
    public boolean followerConnected;
    public double positionRad;
    public double velocityRadPerSec;
    public double appliedVolts;
    public double statorCurrentAmps;
    public double supplyCurrentAmps;
    public double tempCelsius;
    public double followerAppliedVolts;
    public double followerStatorCurrentAmps;
    public double followerSupplyCurrentAmps;
    public double followerTempCelsius;

    public boolean limitSwitchConnected;
    public boolean limitSwitchActivated;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setPosition(double positionRad) {}

  default void setPosition(Angle position) {}

  default void stop() {}

  default void resetPosition(Angle newPosition) {}

  default void setBrake(boolean brake) {}
}
