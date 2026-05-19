package frc.robot.util.io.motors.pivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.io.motors.MotorIO;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Pivot {
  private final String name;
  private final PivotIO io;
  protected final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private MotorIO.MotorIOMode mode;

  private final BooleanSupplier brakeDurNeutral;

  public Pivot(String name, PivotIO io) {
    this(name, io, () -> false);
  }

  public Pivot(String name, PivotIO io, BooleanSupplier brakeMode) {
    this.name = name;
    this.io = io;
    this.mode = brakeMode.getAsBoolean() ? MotorIO.MotorIOMode.BRAKE : MotorIO.MotorIOMode.COAST;
    this.brakeDurNeutral = brakeMode;

    // Initialize input arrays
    inputs.followerConnected = new boolean[io.getNumFollowers()];
    inputs.followerTempCelsius = new double[io.getNumFollowers()];
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void runOpenLoop(double volts) {
    io.setVoltage(volts);
    mode = MotorIO.MotorIOMode.VOLTAGE_CONTROL;
    Logger.recordOutput(name + "/SetpointDeg", -1);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void runClosedLoop(double deg) {
    io.setPosition(deg);
    mode = MotorIO.MotorIOMode.VELOCITY_CONTROL;
    Logger.recordOutput(name + "/SetpointDeg", deg);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public void stop() {
    if (brakeDurNeutral.getAsBoolean()) {
      io.brake();
      mode = MotorIO.MotorIOMode.BRAKE;
    } else {
      io.coast();
      mode = MotorIO.MotorIOMode.COAST;
    }
    Logger.recordOutput(name + "/SetpointDeg", -1);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public Angle getPosition() {
    return Degrees.of(inputs.positionDeg);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }
}
