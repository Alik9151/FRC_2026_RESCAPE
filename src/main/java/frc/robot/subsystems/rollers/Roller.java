package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Roller {
  private final String name;
  private final RollerIO io;
  protected final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private RollerIO.RollerIOMode mode;

  private final BooleanSupplier brakeDurNeutral;

  public Roller(String name, RollerIO io) {
    this(name, io, () -> false);
  }

  public Roller(String name, RollerIO io, BooleanSupplier brakeMode) {
    this.name = name;
    this.io = io;
    this.mode =
        brakeMode.getAsBoolean() ? RollerIO.RollerIOMode.BRAKE : RollerIO.RollerIOMode.COAST;
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
    mode = RollerIO.RollerIOMode.VOLTAGE_CONTROL;
  }

  public void runClosedLoop(double rps) {
    io.setVelocity(rps);
    mode = RollerIO.RollerIOMode.CLOSED_LOOP;
  }

  public void stop() {
    if (brakeDurNeutral.getAsBoolean()) {
      io.brake();
      mode = RollerIO.RollerIOMode.BRAKE;
    } else {
      io.coast();
      mode = RollerIO.RollerIOMode.COAST;
    }
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocityRPS);
  }

  public double getVelocityRPS() {
    return inputs.velocityRPS;
  }
}
