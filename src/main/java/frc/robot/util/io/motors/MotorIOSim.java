package frc.robot.util.io.motors;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Arrays;

public class MotorIOSim implements MotorIO {
  private final int numFollowers;

  protected double appliedVoltage;
  protected final PIDController pid;
  protected boolean isClosedLoop;

  public MotorIOSim(
      DCMotor motorModel, double reduction, double moi, double kP, double kD, int numFollowers) {
    this.numFollowers = numFollowers;
    pid = new PIDController(kP, 0.0, kD);
  }

  protected void updateMotorInputs(MotorIOInputs inputs) {
    inputs.connected = true;
    inputs.appliedVoltage = appliedVoltage;
    inputs.tempCelsius = 0.0;

    Arrays.fill(inputs.followerConnected, true);
    Arrays.fill(inputs.followerTempCelsius, 0.0);
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVoltage = volts;
  }

  @Override
  public void coast() {
    isClosedLoop = false;
    appliedVoltage = 0.0;
  }

  @Override
  public void brake() {
    isClosedLoop = false;
    appliedVoltage = 0.0;
  }

  @Override
  public int getNumFollowers() {
    return numFollowers;
  }
}
