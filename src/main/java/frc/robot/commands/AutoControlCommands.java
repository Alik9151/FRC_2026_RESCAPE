package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.Reef;
import frc.robot.util.Reef.Pole;
import frc.robot.util.RobotUtil;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class AutoControlCommands {
  public enum AutoState {
    IDLE,
    LOADING_START,
    SCORING_START,
    OVERRIDDEN
  }

  public static final PathConstraints CONSTRAINTS =
      new PathConstraints(2.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  @Getter private static AutoState state = AutoState.IDLE;
  @Setter private static Reef reef;
  private static Pole currentPole;

  public static void setState(AutoState newState) {
    state = newState;
    Logger.recordOutput("AutoControl/State", state);
  }

  public static Command driveToReef(Drive drive) {
    return Commands.defer(
        () -> {
          Pose2d targetPose = updateCurrentPole(drive.getPose()).getPose();
          Logger.recordOutput("AutoControl/TargetPose", targetPose);
          Logger.recordOutput("AutoControl/CurrentTask", "SCORE");
          return AutoBuilder.pathfindToPose(targetPose, CONSTRAINTS, 0.0);
        },
        Set.of(drive));
  }

  public static Pole updateCurrentPole(Pose2d currentPose) {
    currentPole = reef.getBestPole(currentPose.getTranslation());
    Logger.recordOutput("AutoControl/CurrentBranch", currentPole.getPose());
    Logger.recordOutput("AutoControl/ScoringLevel", currentPole.getMaxLevel());
    return currentPole;
  }

  public static Command driveToLoading(Drive drive) {
    return Commands.defer(
        () -> {
          Pose2d targetPose = getClosestLoader(drive.getPose().getTranslation());
          Logger.recordOutput("AutoControl/TargetPose", targetPose);
          Logger.recordOutput("AutoControl/CurrentTask", "LOAD");
          return AutoBuilder.pathfindToPose(targetPose, CONSTRAINTS, 0.0);
        },
        Set.of(drive));
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

  public static Command fullAuto(Drive drive, Elevator elevator, Intake intake, Outtake outtake) {
    Command startWithLoad = cycleFromLoad(drive, elevator, intake, outtake);
    Command startWithReef = cycleFromReef(drive, elevator, intake, outtake);
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
      Drive drive, Elevator elevator, Intake intake, Outtake outtake) {
    return Commands.repeatingSequence(
        elevator.stow(),
        intake.intakeCommand().until(outtake::hasGamePiece).deadlineFor(driveToLoading(drive)),
        driveToReef(drive),
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
      Drive drive, Elevator elevator, Intake intake, Outtake outtake) {
    return Commands.repeatingSequence(
        driveToReef(drive),
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
        intake.intakeCommand().until(outtake::hasGamePiece).deadlineFor(driveToLoading(drive)));
  }
}
