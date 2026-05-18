// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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

public class Intake extends SubsystemBase {
  private final Roller roller;
  private final Pivot pivot;

  public Intake() {
    PivotIO pivotIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.INTAKE_PIVOT,
              IntakeConstants.INTAKE_PIVOT_CONFIG);
          case SIM -> new PivotIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(
                  IntakeConstants.INTAKE_ROLLER_GEAR_RATIO,
                  IntakeConstants.INTAKE_ROLLER_MOI,
                  1,
                  0,
                  180,
                  0),
              IntakeConstants.INTAKE_PIVOT_KP,
              IntakeConstants.INTAKE_PIVOT_KD,
              0);
          default -> new PivotIO() {};
        };
    RollerIO rollerIO =
        switch (Constants.currentMode) {
          case REAL -> new MotorIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.INTAKE_ROLLER,
              IntakeConstants.INTAKE_ROLLER_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(
                  IntakeConstants.INTAKE_ROLLER_GEAR_RATIO,
                  IntakeConstants.INTAKE_ROLLER_MOI,
                  0.2,
                  0,
                  0,
                  0),
              IntakeConstants.INTAKE_ROLLER_KP,
              IntakeConstants.INTAKE_ROLLER_KD,
              0);
          default -> new RollerIO() {};
        };
    pivot = new Pivot("Intake/Pivot", pivotIO);
    roller = new Roller("Intake/Roller", rollerIO);
  }

  @Override
  public void periodic() {
    roller.periodic();
    pivot.periodic();
  }

  public void start() {
    roller.runClosedLoop(IntakeConstants.INTAKE_RPS);
    pivot.runClosedLoop(IntakeConstants.INTAKE_POSITION_DEG);
  }

  public void reverse() {
    roller.runClosedLoop(-IntakeConstants.INTAKE_RPS);
  }

  public void stop() {
    roller.stop();
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }

  public double getPositionDeg() {
    return pivot.getPositionDeg();
  }

  public Command intakeCommand() {
    return startEnd(this::start, this::stop);
  }

  public Command reverseCommand() {
    return startEnd(this::reverse, this::stop);
  }
}
