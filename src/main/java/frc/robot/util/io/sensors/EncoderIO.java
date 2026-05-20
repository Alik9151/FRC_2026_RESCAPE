package frc.robot.util.io.sensors;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Rotations;

@FunctionalInterface
public interface EncoderIO {
  @AutoLog
  class EncoderIOInputs {
    public boolean connected = false;
    public Angle position = Rotations.zero();
  }

  void updateInputs(EncoderIOInputs inputs);
}
