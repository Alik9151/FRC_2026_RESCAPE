package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOTalonFXSim extends IntakeIOTalonFX {
  private final FlywheelSim flywheelSim;

  public IntakeIOTalonFXSim() {
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                IntakeConstants.INTAKE_MOI,
                IntakeConstants.INTAKE_GEAR_RATIO),
            DCMotor.getKrakenX60(1));
  }
}
