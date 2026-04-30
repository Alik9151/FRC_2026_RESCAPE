package frc.robot.util.loading;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

public class loadingUtils {

  public static Pose2d getClosestLoader(Translation2d robotPose) {
    double distL =
        robotPose.getSquaredDistance(FieldConstants.LOADING_STATION_LEFT.getTranslation());
    double distR =
        robotPose.getSquaredDistance(FieldConstants.LOADING_STATION_RIGHT.getTranslation());
    if (distL < distR) return FieldConstants.LOADING_STATION_LEFT;
    return FieldConstants.LOADING_STATION_RIGHT;
  }
}
