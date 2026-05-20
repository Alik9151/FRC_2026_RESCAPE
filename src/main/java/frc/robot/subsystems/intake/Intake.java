// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.util.subsystems.RobotStateHandler;

public class Intake extends SubsystemBase {
  private final Pivot pivot;
  private final Roller roller;

  public Intake() {
    PivotIO pivotIO =
        switch (Constants.currentMode) {
          case REAL -> new PivotIOTalonFX(
                  Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
                  Constants.CANConstants.INTAKE_PIVOT,
                  IntakeConstants.PIVOT_CONFIG)
              .useControlRequest(new MotionMagicVoltage(0).withOverrideBrakeDurNeutral(true));
          case SIM -> new PivotIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(
                  IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.ROLLER_MOI, 1, 0, 180, 0),
              IntakeConstants.PIVOT_KP,
              IntakeConstants.PIVOT_KD,
              0);
          default -> new PivotIO() {};
        };
    RollerIO rollerIO =
        switch (Constants.currentMode) {
          case REAL -> new RollerIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.INTAKE_ROLLER,
              IntakeConstants.ROLLER_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              new MotorIO.MechanismConstraints(
                  IntakeConstants.ROLLER_GEAR_RATIO, IntakeConstants.ROLLER_MOI, 0.2, 0, 0, 0),
              IntakeConstants.ROLLER_KP,
              IntakeConstants.ROLLER_KD,
              0);
          default -> new RollerIO() {};
        };

    pivot = new Pivot("Intake/Pivot", pivotIO, RobotStateHandler::isEnabled);
    roller = new Roller("Intake/Roller", rollerIO);
  }

  @Override
  public void periodic() {
    pivot.periodic();
    roller.periodic();
  }

  public void start() {
    pivot.runClosedLoop(IntakeConstants.ENGAGED_DEG);
    roller.runClosedLoop(IntakeConstants.RPS);
  }

  public void reverse() {
    roller.runClosedLoop(-IntakeConstants.RPS);
  }

  public void stop() {
    roller.stop();
  }

  public double getPositionDeg() {
    return pivot.getPositionDeg();
  }

  public double getVelocityRPS() {
    return roller.getVelocityRPS();
  }

  public Command intakeCommand() {
    return startEnd(this::start, this::stop);
  }
}
