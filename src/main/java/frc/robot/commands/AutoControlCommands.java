package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ForeignRobot;
import frc.robot.util.Reef;
import frc.robot.util.Reef.Pole;
import frc.robot.util.RobotUtil;
import java.util.ArrayList;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class AutoControlCommands {
  public enum AutoState {
    IDLE,
    OVERRIDDEN,
    // drive to pose (manually triggered)
    DTP_LOAD,
    DTP_REEF,
    // full auto
    LOADING_START,
    SCORING_START,
  }

  private static final double MAX_FOREIGN_ROBOT_ERROR_SQUARED = 0.1 * 0.1; // meters
  private static final double MAX_ROBOT_AGE = 0.2;
  private static final ArrayList<ForeignRobot> foreignRobots =
      new ArrayList<>(8); // 8 is a lucky number

  @Getter private static AutoState state = AutoState.IDLE;
  @Setter private static Reef reef;
  private static Pole currentPole;

  public static void setState(AutoState newState) {
    state = newState;
    Logger.recordOutput("AutoControl/State", state);
  }

  public static Pole updateCurrentPole(Pose2d currentPose) {
    currentPole = reef.getBestPole(currentPose.getTranslation());
    Logger.recordOutput("AutoControl/CurrentBranch", currentPole.getPose());
    Logger.recordOutput("AutoControl/ScoringLevel", currentPole.getMaxLevel());
    return currentPole;
  }

  private static Pose2d getClosestLoader(Translation2d robotPose) {
    Pose2d leftLoadingStation;
    Pose2d rightLoadingStation;

    if (RobotUtil.isRedAlliance()) {
      leftLoadingStation = FieldConstants.LOADING_STATION_LEFT_RED;
      rightLoadingStation = FieldConstants.LOADING_STATION_RIGHT_RED;
    } else {
      leftLoadingStation = FieldConstants.LOADING_STATION_LEFT_BLUE;
      rightLoadingStation = FieldConstants.LOADING_STATION_RIGHT_BLUE;
    }

    leftLoadingStation =
        new Pose2d(
            leftLoadingStation
                .getTranslation()
                .plus(
                    FieldConstants.LOADING_TRANSLATION.rotateBy(leftLoadingStation.getRotation())),
            leftLoadingStation.getRotation());

    rightLoadingStation =
        new Pose2d(
            rightLoadingStation
                .getTranslation()
                .plus(
                    FieldConstants.LOADING_TRANSLATION.rotateBy(rightLoadingStation.getRotation())),
            rightLoadingStation.getRotation());

    double distL = robotPose.getSquaredDistance(leftLoadingStation.getTranslation());
    double distR = robotPose.getSquaredDistance(rightLoadingStation.getTranslation());
    if (distL < distR) return leftLoadingStation;
    return rightLoadingStation;
  }

  public static Command driveToReef(Drive drive, Vision vision) {
    Supplier<Pose2d> getTarget = () -> updateCurrentPole(drive.getPose()).getPose();
    return new driveToPointWithObstaclesCommand(getTarget, drive, vision)
        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoControl/CurrentTask", "SCORE")));
  }

  public static Command driveToLoading(Drive drive, Vision vision) {
    Supplier<Pose2d> getTarget = () -> getClosestLoader(drive.getPose().getTranslation());
    return new driveToPointWithObstaclesCommand(getTarget, drive, vision)
        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoControl/CurrentTask", "LOAD")));
  }

  public static Command fullAuto(
      Drive drive, Vision vision, Elevator elevator, Intake intake, Outtake outtake) {
    Command startWithLoad = cycleFromLoad(drive, vision, elevator, intake, outtake);
    Command startWithReef = cycleFromReef(drive, vision, elevator, intake, outtake);
    return Commands.deferredProxy(
            () ->
                switch (state) {
                  case LOADING_START -> startWithLoad;
                  case SCORING_START -> startWithReef;
                  default -> {
                    if (outtake.hasGamePiece()) {
                      setState(AutoState.SCORING_START);
                      yield startWithReef;
                    }
                    setState(AutoState.LOADING_START);
                    yield startWithLoad;
                  }
                })
        .finallyDo(() -> setState(AutoState.IDLE));
  }

  private static Command cycleFromLoad(
      Drive drive, Vision vision, Elevator elevator, Intake intake, Outtake outtake) {
    return Commands.repeatingSequence(
        elevator.stow(),
        intake
            .intakeCommand()
            .until(outtake::hasGamePiece)
            .deadlineFor(driveToLoading(drive, vision)),
        driveToReef(drive, vision),
        Commands.runOnce(drive::stopWithX, drive),
        Commands.runOnce(
            () -> elevator.setState(Elevator.toElevatorState(currentPole.getMaxLevel())), elevator),
        Commands.waitUntil(elevator::hasReachedSetpoint),
        Commands.runOnce(() -> outtake.runPivot(currentPole.getMaxLevel() == 4), outtake),
        Commands.waitUntil(outtake::hasReachedSetpoint),
        Commands.runOnce(outtake::startRoller, outtake),
        Commands.waitUntil(() -> !outtake.hasGamePiece())
            .finallyDo(() -> currentPole.updateLevel(currentPole.getMaxLevel())),
        Commands.runOnce(outtake::stop, outtake));
  }

  private static Command cycleFromReef(
      Drive drive, Vision vision, Elevator elevator, Intake intake, Outtake outtake) {
    return Commands.repeatingSequence(
        driveToReef(drive, vision),
        Commands.runOnce(drive::stopWithX, drive),
        Commands.runOnce(
            () -> elevator.setState(Elevator.toElevatorState(currentPole.getMaxLevel())), elevator),
        Commands.waitUntil(elevator::hasReachedSetpoint),
        Commands.runOnce(() -> outtake.runPivot(currentPole.getMaxLevel() == 4), outtake),
        Commands.waitUntil(outtake::hasReachedSetpoint),
        Commands.runOnce(outtake::startRoller, outtake),
        Commands.waitUntil(() -> !outtake.hasGamePiece())
            .finallyDo(() -> currentPole.updateLevel(currentPole.getMaxLevel())),
        Commands.runOnce(outtake::stop, outtake),
        elevator.stow(),
        intake
            .intakeCommand()
            .until(outtake::hasGamePiece)
            .deadlineFor(driveToLoading(drive, vision)));
  }
}
