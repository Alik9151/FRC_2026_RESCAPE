package frc.robot.subsystems.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class OuttakeIOSim extends OuttakeIOTalonFX {
    private final FlywheelSim flywheelSim;

  private final PIDController outtakePID;

  private double outtakeVolts;
  private boolean isClosedLoopOuttake;


  public OuttakeIOSim() {
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                OuttakeConstants.OUTTAKE_MOI,
                OuttakeConstants.OUTTAKE_GEAR_RATIO),
            DCMotor.getKrakenX60(1));
    outtakePID = new PIDController(
      OuttakeConstants.OUTTAKE_KP * 125.0,
      OuttakeConstants.OUTTAKE_KI,
      OuttakeConstants.OUTTAKE_KD
    );

  }

  public void updateInputs(OuttakeIOInputs inputs) {
    if (isClosedLoopOuttake) {
      outtakeVolts = outtakePID.calculate(flywheelSim.getAngularVelocityRPM());
    }

    flywheelSim.setInput(outtakeVolts);
    flywheelSim.update(0.02);

    inputs.connected = true;
    inputs.appliedVolts = outtakeVolts;
    inputs.statorCurrentAmps = flywheelSim.getCurrentDrawAmps();
    inputs.velocityRPS = flywheelSim.getAngularVelocityRPM() / 60.0;
  }

  /** Sets the motor's speed given an RPS input */
  public void setVelocity(double rps) {
    isClosedLoopOuttake = true;
    outtakePID.setSetpoint(rps * 60.0);
  }

  /** Stop the motor */
  public void stop() {
    isClosedLoopOuttake = false;
    outtakeVolts = 0.0;
  }
}
