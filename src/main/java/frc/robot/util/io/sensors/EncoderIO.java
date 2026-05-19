package frc.robot.util.io.sensors;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
  @AutoLog
  class EncoderIOInputs {
    public boolean connected = false;
    public Angle position = Rotations.zero();
  }

  default void updateInputs(EncoderIOInputs inputs) {}
}
