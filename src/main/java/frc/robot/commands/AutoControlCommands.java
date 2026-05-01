package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.reef.Reef;
import frc.robot.util.reef.Reef.Pole;
import java.util.Set;
import java.util.function.Supplier;

public class AutoControlCommands {
  public static boolean autoControl = true;
  public static final PathConstraints constraints =
      new PathConstraints(2.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  //   public static Command driveToReef(Translation2d robotPose, Reef reef) {
  //     Pose2d targetPose = reef.getBestPole(robotPose).getPose2d();
  //     Command pathfind = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  //     return pathfind;
  //   }

  //   public static Command driveToLoading(Translation2d robotPose, Reef reef) {
  //     Pose2d targetPose = getClosestLoader(robotPose);
  //     Command pathfind = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  //     return pathfind;
  //   }

  public static Command driveToReef(Supplier<Pose2d> robotPoseSupplier, Reef reef) {
    Pole bestPole = reef.getBestPole(robotPoseSupplier.get().getTranslation());
    bestPole.updateLevel(bestPole.getMaxLevel());
    Pose2d targetPose = bestPole.getPose2d();
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0)
        .andThen(
            Commands.defer(
                () -> {
                  if (autoControl) {
                    return driveToLoading(robotPoseSupplier, reef);
                  }
                  return Commands.none();
                },
                Set.of()));
  }

  public static Command driveToLoading(Supplier<Pose2d> robotPoseSupplier, Reef reef) {
    Pose2d targetPose = getClosestLoader(robotPoseSupplier.get().getTranslation());
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0)
        .andThen(
            Commands.defer(
                () -> {
                  if (autoControl) {
                    return driveToReef(robotPoseSupplier, reef);
                  }
                  return Commands.none();
                },
                Set.of()));
  }

  public static Pose2d getClosestLoader(Translation2d robotPose) {
    double distL =
        robotPose.getSquaredDistance(FieldConstants.LOADING_STATION_LEFT.getTranslation());
    double distR =
        robotPose.getSquaredDistance(FieldConstants.LOADING_STATION_RIGHT.getTranslation());
    if (distL < distR) return FieldConstants.LOADING_STATION_LEFT;
    return FieldConstants.LOADING_STATION_RIGHT;
  }
}
