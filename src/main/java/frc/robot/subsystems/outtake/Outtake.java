// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  private final Debouncer debouncer = new Debouncer(0.1);

  public Outtake(OuttakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);
  }

  public void enable() {
    io.setVelocity(OuttakeConstants.OUTTAKE_RPS);
  }

  public void reverse() {
    io.setVelocity(-OuttakeConstants.OUTTAKE_RPS);
  }

  public void stop() {
    io.stop();
  }

  public boolean hasGamePiece() {
    return debouncer.calculate(inputs.isLoaded);
  }

  public Command outtakeCommand() {
    return startEnd(this::enable, this::stop);
  }

  public Command reverseCommand() {
    return startEnd(this::reverse, this::stop);
  }
}
