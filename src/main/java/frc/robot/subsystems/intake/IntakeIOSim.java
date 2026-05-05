package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim extends IntakeIOTalonFX {

  private final FlywheelSim flywheelSim;

  private final PIDController intakePID;

  private double intakeVolts;
  private boolean isClosedLoopIntake;


  public IntakeIOSim() {
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                IntakeConstants.INTAKE_MOI,
                IntakeConstants.INTAKE_GEAR_RATIO),
            DCMotor.getKrakenX60(1));
    intakePID = new PIDController(
      IntakeConstants.INTAKE_KP * 125.0,
      IntakeConstants.INTAKE_KI,
      IntakeConstants.INTAKE_KD
    );

  }

  public void updateInputs(IntakeIOInputs inputs) {
    if (isClosedLoopIntake) {
      intakeVolts = intakePID.calculate(flywheelSim.getAngularVelocityRPM());
    }

    flywheelSim.setInput(intakeVolts);
    flywheelSim.update(0.02);

    inputs.connected = true;
    inputs.appliedVolts = intakeVolts;
    inputs.statorCurrentAmps = flywheelSim.getCurrentDrawAmps();
    inputs.velocityRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
  }

  /** Sets the motor's speed given an RPS input */
  public void setVelocity(double rps) {
    isClosedLoopIntake = true;
    intakePID.setSetpoint(rps * 60.0);
  }

  /** Stop the motor */
  public void stop() {
    isClosedLoopIntake = false;
    intakeVolts = 0.0;
  }
}
