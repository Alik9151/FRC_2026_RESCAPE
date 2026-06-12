package frc.robot.commands;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.util.RobotUtil;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.SimulatedArena;
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

  public static void setState(AutoState newState) {
    state = newState;
    Logger.recordOutput("AutoControl/State", state);
  }

  public static List<Pose2d> updateCurrentPole(Pose2d currentPose) {
    int level = reef.getLevel();
    List<Pose2d> currentPoles = reef.getPoles(level);
    Logger.recordOutput("AutoControl/CurrentBranches", currentPoles.toArray(new Pose2d[0]));
    Logger.recordOutput("AutoControl/ScoringLevel", level);
    return currentPoles;
  }

  private static List<Pose2d> getLoaders(Translation2d robotPose) {
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
    return List.of(leftLoadingStation, rightLoadingStation);
  }

  private static void updateObstacles(Drive drive, Vision vision) {
    Translation2d[] robotTranslations = vision.getForeignRobotTranslations(drive.getPose());

    double currentTime = Timer.getTimestamp();

    foreignRobots.removeIf(
        robot -> {
          robot.isVisible = false;
          return currentTime - robot.getTimestamp() > MAX_ROBOT_AGE;
        });

    for (ForeignRobot foreignRobot : foreignRobots) {
      int indexToUpdate = -1;
      double min = MAX_FOREIGN_ROBOT_ERROR_SQUARED;
      for (int i = 0; i < robotTranslations.length; i++) {
        if (robotTranslations[i] != null) {
          double distance = foreignRobot.getSquaredDistance(robotTranslations[i]);
          if (distance < min) {
            min = distance;
            indexToUpdate = i;
          }
        }
      }
      if (indexToUpdate != -1) {
        // if velocity wrong look at timestamp if not we're geniuses
        foreignRobot.updateTranslation(robotTranslations[indexToUpdate], currentTime);
        foreignRobot.isVisible = true;
        robotTranslations[indexToUpdate] = null;
      }
    }

    // leftovers get made into new foreign robots

    for (Translation2d robotTranslation : robotTranslations) {
      if (robotTranslation != null) {
        foreignRobots.add(new ForeignRobot(currentTime, robotTranslation));
      }
    }

    ArrayList<Pair<Translation2d, Translation2d>> obstacleCorners =
        new ArrayList<>(robotTranslations.length);
    for (ForeignRobot robot : foreignRobots) {
      if (robot.isVisible) {
        obstacleCorners.add(robot.getPredictedCorners());
      }
    }
    Pathfinding.setDynamicObstacles(obstacleCorners, drive.getPose().getTranslation());
  }

  public static Command driveToReef(Drive drive, Vision vision) {
    return drive
        .driveToBestPose(() -> updateCurrentPole(drive.getPose()))
        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoControl/CurrentTask", "REEF")))
        .deadlineFor(Commands.run(() -> updateObstacles(drive, vision)));
  }

  public static Command driveToLoading(Drive drive, Vision vision) {
    return drive
        .driveToBestPose(() -> getLoaders(drive.getPose().getTranslation()))
        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoControl/CurrentTask", "LOAD")))
        .deadlineFor(Commands.run(() -> updateObstacles(drive, vision)));
  }

  // These next two commands are just for fun!
  public static Command driveToCoral(Drive drive, Vision vision) {
    return drive
        .driveToBestPose(() -> getCoralOrLoader(drive, vision))
        .alongWith(Commands.runOnce(() -> Logger.recordOutput("AutoControl/CurrentTask", "LOAD")))
        .deadlineFor(Commands.run(() -> updateObstacles(drive, vision)));
  }

  public static List<Pose2d> getCoralOrLoader(Drive drive, Vision vision) {
    Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    List<Pose2d> coralPose2ds = new ArrayList<>();
    for (Pose3d coral : coralPoses) {
      Translation2d coralTranslation = coral.getTranslation().toTranslation2d();
      Rotation2d coralRotation =
          coralTranslation
              .minus(drive.getPose().getTranslation())
              .getAngle()
              .rotateBy(Rotation2d.k180deg);
      Translation2d offset = new Translation2d(0.35, coralRotation);
      coralTranslation = coralTranslation.plus(offset);
      coralPose2ds.add(new Pose2d(coralTranslation, coralRotation));
    }

    if (coralPose2ds.isEmpty()) {
      return getLoaders(drive.getPose().getTranslation());
    }
    return coralPose2ds;
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
            // .deadlineFor(driveToLoading(drive, vision)),
            .deadlineFor(driveToCoral(drive, vision)), // har har funny command please work
        driveToReef(drive, vision),
        Commands.runOnce(drive::stopWithX, drive),
        Commands.runOnce(
            () -> elevator.setState(Elevator.toElevatorState(reef.getLevel())), elevator),
        Commands.waitUntil(elevator::hasReachedSetpoint),
        Commands.runOnce(() -> outtake.runPivot(reef.getLevel() == 4), outtake),
        Commands.waitUntil(outtake::hasReachedSetpoint),
        Commands.runOnce(outtake::startRoller, outtake),
        Commands.waitUntil(() -> !outtake.hasGamePiece())
            .finallyDo(() -> reef.updatePole(reef.getLevel(), drive.pathfinder.getGoalPose())),
        Commands.runOnce(outtake::stop, outtake));
  }

  private static Command cycleFromReef(
      Drive drive, Vision vision, Elevator elevator, Intake intake, Outtake outtake) {
    return Commands.repeatingSequence(
        driveToReef(drive, vision),
        Commands.runOnce(drive::stopWithX, drive),
        Commands.runOnce(
            () -> elevator.setState(Elevator.toElevatorState(reef.getLevel())), elevator),
        Commands.waitUntil(elevator::hasReachedSetpoint),
        Commands.runOnce(() -> outtake.runPivot(reef.getLevel() == 4), outtake),
        Commands.waitUntil(outtake::hasReachedSetpoint),
        Commands.runOnce(outtake::startRoller, outtake),
        Commands.waitUntil(() -> !outtake.hasGamePiece())
            .finallyDo(() -> reef.updatePole(reef.getLevel(), drive.pathfinder.getGoalPose())),
        Commands.runOnce(outtake::stop, outtake),
        elevator.stow(),
        intake
            .intakeCommand()
            .until(outtake::hasGamePiece)
            // .deadlineFor(driveToLoading(drive, vision)));
            .deadlineFor(driveToCoral(drive, vision))); // har har funny command please work
  }
}
