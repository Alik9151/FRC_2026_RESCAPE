package frc.robot.util.io.sensors;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.*;

public class CoralSensorIOLaserCan implements CoralSensorIO {
  private final LaserCan laserCan;

  public CoralSensorIOLaserCan(int id) {
    this(
        id, new RegionOfInterest(8, 8, 16, 16), RangingMode.SHORT, TimingBudget.TIMING_BUDGET_20MS);
  }

  public CoralSensorIOLaserCan(
      int id, RegionOfInterest roi, RangingMode rangingMode, TimingBudget timingBudget) {
    laserCan = new LaserCan(id);
    try {
      laserCan.setRegionOfInterest(roi);
      laserCan.setRangingMode(rangingMode);
      laserCan.setTimingBudget(timingBudget);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(CoralSensorIOInputs inputs) {
    var measurement = laserCan.getMeasurement();
    inputs.valid =
        measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    if (inputs.valid) {
      inputs.distanceMillimeters = measurement.distance_mm;
    } else {
      inputs.distanceMillimeters = 0;
    }
  }
}
