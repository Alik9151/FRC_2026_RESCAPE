package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.metersToRadians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim elevatorSim;

  private final PIDController elevatorPID;

  private double elevatorVolts;
  private boolean isClosedLoopElevator;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            ElevatorConstants.ELEVATOR_GEAR_RATIO,
            ElevatorConstants.CARRIAGE_MASS,
            ElevatorConstants.DRUM_RADIUS,
            0,
            ElevatorConstants.MAX_HEIGHT,
            true,
            0);
    elevatorPID =
        new PIDController(
            ElevatorConstants.ELEVATOR_CONFIG.Slot0.kP * 125.0,
            ElevatorConstants.ELEVATOR_CONFIG.Slot0.kI,
            ElevatorConstants.ELEVATOR_CONFIG.Slot0.kD);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoopElevator) {
      elevatorVolts = elevatorPID.calculate(inputs.positionRad);
    }

    elevatorVolts = MathUtil.clamp(elevatorVolts, -12.0, 12.0);

    elevatorSim.setInputVoltage(elevatorVolts);
    elevatorSim.update(0.02);

    double positionMeters = elevatorSim.getPositionMeters();

    inputs.leaderConnected = true;
    inputs.followerConnected = true;

    inputs.positionRad = metersToRadians(positionMeters);
    inputs.velocityRadPerSec = metersToRadians(elevatorSim.getVelocityMetersPerSecond());

    inputs.appliedVolts = elevatorVolts;
    inputs.statorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0;

    inputs.followerAppliedVolts = elevatorVolts;
    inputs.followerStatorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.followerSupplyCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.followerTempCelsius = 25.0;

    inputs.limitSwitchConnected = true;
    inputs.limitSwitchActivated = elevatorSim.getPositionMeters() <= 0.001;
  }

  public void setOpenLoop(double output) {
    isClosedLoopElevator = false;
    elevatorVolts = output * 12.0;
  }

  public void setPosition(double positionRad) {
    isClosedLoopElevator = true;
    elevatorPID.setSetpoint(positionRad);
  }

  public void setPosition(Angle position) {
    setPosition(position.in(Units.Radians));
  }

  public void stop() {
    isClosedLoopElevator = false;
    elevatorVolts = 0.0;
  }

  public void resetPosition(Angle newPosition) {
    elevatorSim.setState(newPosition.in(Units.Radians) * ElevatorConstants.DRUM_RADIUS, 0.0);
    elevatorPID.reset();
  }

  public void setBrake(boolean brake) {}
}
