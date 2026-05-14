package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Arrays;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox;
  private final int numFollowers;

  private double appliedVoltage = 0.0;
  private final PIDController pid;
  private boolean isClosedLoop;

  public RollerIOSim(
      DCMotor motorModel, double reduction, double moi, double kP, double kD, int numFollowers) {
    gearbox = motorModel;
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
    pid = new PIDController(kP, 0.0, kD);
    this.numFollowers = numFollowers;
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (isClosedLoop) {
      appliedVoltage =
          MathUtil.clamp(
              pid.calculate(sim.getAngularVelocity().in(RotationsPerSecond)), -12.0, 12.0);
      sim.setInputVoltage(appliedVoltage);
    }

    sim.update(0.02);

    inputs.connected = true;
    inputs.velocityRPS = sim.getAngularVelocity().in(RotationsPerSecond);
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.statorCurrentAmps =
        gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage);
    inputs.tempCelsius = 0.0;

    Arrays.fill(inputs.followerConnected, true);
    Arrays.fill(inputs.followerTempCelsius, 0.0);
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    sim.setInputVoltage(volts);
    appliedVoltage = volts;
  }

  @Override
  public void setVelocity(double rps) {
    pid.setSetpoint(rps);
    isClosedLoop = true;
  }

  @Override
  public void coast() {
    isClosedLoop = false;
    sim.setInputVoltage(0.0);
  }

  @Override
  public void brake() {
    isClosedLoop = false;
    sim.setInputVoltage(0.0);
  }

  @Override
  public int getNumFollowers() {
    return numFollowers;
  }
}
