// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.io.motors.*;
import frc.robot.util.io.motors.pivot.Pivot;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.pivot.PivotIOSim;
import frc.robot.util.io.motors.pivot.PivotIOTalonFX;
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.motors.roller.RollerIOTalonFX;
import frc.robot.util.io.sensors.CoralSensorIO;
import frc.robot.util.io.sensors.CoralSensorIOInputsAutoLogged;
import frc.robot.util.subsystems.RobotStateHandler;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final Pivot pivot;
  private final Roller roller;
  @Setter private CoralSensorIO sensor = new CoralSensorIO() {};
  private final CoralSensorIOInputsAutoLogged sensorInputs = new CoralSensorIOInputsAutoLogged();

  private double setpointDeg;

  private final Debouncer debouncer = new Debouncer(0.1);

  public Outtake() {
    PivotIO pivotIO =
        switch (Constants.currentMode) {
          case REAL -> new PivotIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.OUTTAKE_PIVOT,
              OuttakeConstants.PIVOT_CONFIG);
          case SIM -> new PivotIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(
                  OuttakeConstants.ROLLER_GEAR_RATIO, OuttakeConstants.ROLLER_MOI, 1, 0, 180, 0),
              OuttakeConstants.PIVOT_KP,
              OuttakeConstants.PIVOT_KD,
              0);
          default -> new PivotIO() {};
        };
    RollerIO rollerIO =
        switch (Constants.currentMode) {
          case REAL -> new RollerIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.OUTTAKE_ROLLER,
              OuttakeConstants.ROLLER_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(
                  OuttakeConstants.ROLLER_GEAR_RATIO, OuttakeConstants.ROLLER_MOI, 0.2, 0, 0, 0),
              OuttakeConstants.ROLLER_KP,
              OuttakeConstants.ROLLER_KD,
              0);
          default -> new RollerIO() {};
        };

    pivot = new Pivot("Outtake/Pivot", pivotIO, RobotStateHandler::isEnabled);
    roller = new Roller("Outtake/Roller", rollerIO);
  }

  @Override
  public void periodic() {
    pivot.periodic();
    roller.periodic();
    sensor.updateInputs(sensorInputs);
    Logger.processInputs("Outtake/CoralSensor", sensorInputs);
  }

  public void runPivot(boolean isL4) {
    setpointDeg = isL4 ? OuttakeConstants.DROPPING_DEG_L4 : OuttakeConstants.DROPPING_DEG;
    pivot.runClosedLoop(setpointDeg);
  }

  public void startRoller() {
    roller.runClosedLoop(OuttakeConstants.RPS);
  }

  public void reverse() {
    roller.runClosedLoop(-OuttakeConstants.RPS);
  }

  public void stopRoller() {
    roller.stop();
  }

  public void stop() {
    setpointDeg = OuttakeConstants.STOWED_DEG;
    pivot.runClosedLoop(setpointDeg);
    roller.stop();
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }

  public double getPositionDeg() {
    return pivot.getPositionDeg();
  }

  public boolean hasReachedSetpoint() {
    return Math.abs(pivot.getPositionDeg() - setpointDeg) < 3.0;
  }

  public boolean hasGamePiece() {
    return debouncer.calculate(sensorInputs.valid && sensorInputs.distanceMillimeters < 100);
  }

  public Command outtakeCommand() {
    return startEnd(this::startRoller, this::stopRoller);
  }
}
