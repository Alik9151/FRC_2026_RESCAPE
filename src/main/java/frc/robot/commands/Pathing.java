package frc.robot.commands;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ForeignRobot;
import java.util.ArrayList;
import java.util.function.Supplier;

public class Pathing {
  private static final double MAX_FOREIGN_ROBOT_ERROR_SQUARED = 0.1 * 0.1; // meters
  private static final double MAX_ROBOT_AGE = 0.2;
  private static final ArrayList<ForeignRobot> foreignRobots = new ArrayList<>(8);

  private static Command pathfindingCommand;
  private static Pose2d currentTarget;

  public static Command driveToPointWithObstacles(
      Supplier<Pose2d> targetPose, Drive drive, Vision vision) {
    return new FunctionalCommand(
        () -> {
          currentTarget = targetPose.get();
          CommandScheduler.getInstance().schedule(generatePathfindingCommand(drive, vision));
        },
        () -> {
          Pose2d newTarget = targetPose.get();
          if (newTarget.equals(currentTarget)) return;
          if (pathfindingCommand != null) {
            pathfindingCommand.cancel();
          }
          currentTarget = newTarget;
          CommandScheduler.getInstance().schedule(generatePathfindingCommand(drive, vision));
        },
        interrupted -> {
          if (pathfindingCommand != null) {
            pathfindingCommand.cancel();
          }
        },
        () -> pathfindingCommand == null,
        drive);
  }

  private static Command generatePathfindingCommand(Drive drive, Vision vision) {
    return pathfindingCommand =
        drive
            .driveToPose(currentTarget)
            .deadlineFor(Commands.run(() -> updateObstacles(drive, vision)))
            .andThen(() -> pathfindingCommand = null);
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
}
