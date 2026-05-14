package frc.robot.util.io.motors.pivot;

import frc.robot.util.io.motors.MotorIO;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO extends MotorIO {
  @AutoLog
  class PivotIOInputs extends MotorIO.MotorIOInputs {
    public double positionDeg;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setPosition(double deg) {}
}
