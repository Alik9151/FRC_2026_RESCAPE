package frc.robot.subsystems.sensors;

import frc.robot.util.SuperstructureSim;

public class CoralSensorIOSim implements CoralSensorIO {
  private final SuperstructureSim sim;

  public CoralSensorIOSim(SuperstructureSim sim) {
    this.sim = sim;
  }

  @Override
  public void updateInputs(CoralSensorIOInputs inputs) {
    inputs.valid = true;
    inputs.distanceMillimeters = sim.isLoaded() ? 1 : 200;
  }
}
