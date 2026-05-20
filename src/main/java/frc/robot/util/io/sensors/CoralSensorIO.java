package frc.robot.util.io.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface CoralSensorIO {
  @AutoLog
  class CoralSensorIOInputs {
    public boolean valid;
    public int distanceMillimeters;
  }

  default void updateInputs(CoralSensorIOInputs inputs) {}
}
