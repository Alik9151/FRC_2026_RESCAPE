package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ControllerConstants.OPERATOR_DEADBAND;
import static frc.robot.subsystems.elevator.ElevatorConstants.SETPOINTS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.subsystems.ExtendedSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends ExtendedSubsystem {
  public static Angle distanceToAngle(double meters) {
    // fill in with real formula
    return Radians.of(meters);
  }

  public enum ElevatorState {
    STOWED,
    CORAL_L1,
    CORAL_L2,
    CORAL_L3,
    CORAL_L4,
    HOMING,
    MANUAL_CONTROL
  }

  private ElevatorState setpoint;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private boolean elevatorSafetyEngaged;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void disable() {
    if (DriverStation.isEStopped()) io.setOpenLoop(0);
    else io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setState(ElevatorState newState) {
    switch (newState) {
      case HOMING:
      case MANUAL_CONTROL:
        break;
      default:
        io.setPosition(SETPOINTS.get(setpoint));
    }

    setpoint = newState;
    Logger.recordOutput("Elevator/ElevatorState", setpoint);
  }

  public Command homingSequence() {
    Debouncer homingDebouncer = new Debouncer(0.1);
    Timer homingTimer = new Timer();

    return startRun(
            () -> {
              setState(ElevatorState.HOMING);
              io.setOpenLoop(-ElevatorConstants.HOMING_VOLTAGE);
            },
            () -> {
              if (homingDebouncer.calculate(
                      inputs.velocityRadPerSec <= ElevatorConstants.HOMING_VELOCITY_THRESHOLD)
                  && !homingTimer.isRunning()) {
                io.setOpenLoop(0);
                io.resetPosition(Rotations.of(0));
                // wait for operation to finish
                homingTimer.start();
              }
            })
        .until(() -> homingTimer.hasElapsed(0.101))
        .finallyDo(
            () -> {
              homingTimer.stop();
              homingTimer.reset();
              setState(ElevatorState.STOWED);
            });
  }

  public Command manualControl(DoubleSupplier joystick) {
    return run(
        () -> {
          double magnitude = MathUtil.applyDeadband(joystick.getAsDouble(), OPERATOR_DEADBAND);
          if (magnitude > 0.0) {
            setState(ElevatorState.MANUAL_CONTROL);
            io.setOpenLoop(magnitude * ElevatorConstants.MAX_MANUAL_VOLTAGE);
          }
        });
  }
}
