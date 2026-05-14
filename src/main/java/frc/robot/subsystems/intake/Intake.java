// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.Roller;
import frc.robot.subsystems.rollers.RollerIO;
import frc.robot.subsystems.rollers.RollerIOSim;
import frc.robot.subsystems.rollers.RollerIOTalonFX;

public class Intake extends SubsystemBase {
  private final Roller roller;

  public Intake() {
    RollerIO io =
        switch (Constants.currentMode) {
          case REAL -> new RollerIOTalonFX(
              Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
              Constants.CANConstants.INTAKE,
              IntakeConstants.INTAKE_CONFIG);
          case SIM -> new RollerIOSim(
              DCMotor.getKrakenX60(1),
              IntakeConstants.INTAKE_GEAR_RATIO,
              IntakeConstants.INTAKE_MOI,
              IntakeConstants.INTAKE_KP * 10,
              IntakeConstants.INTAKE_KD,
              0);
          default -> new RollerIO() {};
        };
    roller = new Roller("Intake", io);
  }

  @Override
  public void periodic() {
    roller.periodic();
  }

  public void start() {
    roller.runClosedLoop(IntakeConstants.INTAKE_RPS);
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

  public Command intakeCommand() {
    return startEnd(this::start, this::stop);
  }

  public Command reverseCommand() {
    return startEnd(this::reverse, this::stop);
  }
}
