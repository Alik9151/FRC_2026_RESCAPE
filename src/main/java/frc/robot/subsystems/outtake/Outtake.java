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
import frc.robot.util.io.motors.roller.Roller;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.motors.roller.RollerIOSim;
import frc.robot.util.io.sensors.CoralSensorIO;
import frc.robot.util.io.sensors.CoralSensorIOInputsAutoLogged;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final Roller roller;
  private final Pivot pivot;
  @Setter private CoralSensorIO sensor = new CoralSensorIO() {};
  private final CoralSensorIOInputsAutoLogged sensorInputs = new CoralSensorIOInputsAutoLogged();

  private final Debouncer debouncer = new Debouncer(0.1);

  public Outtake() {
    PivotIO pivotIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.OUTTAKE_PIVOT,
              OuttakeConstants.OUTTAKE_PIVOT_CONFIG);
          case SIM -> new PivotIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(
                  OuttakeConstants.OUTTAKE_ROLLER_GEAR_RATIO,
                  OuttakeConstants.OUTTAKE_ROLLER_MOI,
                  1,
                  0,
                  180,
                  0),
              OuttakeConstants.OUTTAKE_PIVOT_KP,
              OuttakeConstants.OUTTAKE_PIVOT_KD,
              0);
          default -> new PivotIO() {};
        };

    RollerIO rollerIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.OUTTAKE_ROLLER,
              OuttakeConstants.OUTTAKE_ROLLER_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(
                  OuttakeConstants.OUTTAKE_ROLLER_GEAR_RATIO,
                  OuttakeConstants.OUTTAKE_ROLLER_MOI,
                  0.2,
                  0,
                  0,
                  0),
              OuttakeConstants.OUTTAKE_ROLLER_KP,
              OuttakeConstants.OUTTAKE_ROLLER_KD,
              0);
          default -> new RollerIO() {};
        };
    pivot = new Pivot("Outtake/Pivot", pivotIO);
    roller = new Roller("Outtake/Roller", rollerIO, () -> true);
  }

  @Override
  public void periodic() {
    roller.periodic();
    sensor.updateInputs(sensorInputs);
    Logger.processInputs("Outtake/CoralSensor", sensorInputs);
  }

  public void start() {
    roller.runClosedLoop(OuttakeConstants.OUTTAKE_RPS);
  }

  public void reverse() {
    roller.runClosedLoop(-OuttakeConstants.OUTTAKE_RPS);
  }

  public void stop() {
    roller.stop();
    pivot.runClosedLoop(OuttakeConstants.OUTTAKE_STORED_DEG);
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }

  public double getPositionDeg() {
    return pivot.getPositionDeg();
  }

  public boolean hasGamePiece() {
    return debouncer.calculate(sensorInputs.valid && sensorInputs.distanceMillimeters < 100);
  }

  public Command outtakeCommand() {
    return startEnd(this::start, this::stop);
  }

  public Command reverseCommand() {
    return startEnd(this::reverse, this::stop);
  }
}
