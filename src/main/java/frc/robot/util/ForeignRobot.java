package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ForeignRobot {
  private static final double ROBOT_RADII = Units.inchesToMeters(16 * 2);
  private static final Translation2d CORNER_OFFSET = new Translation2d(ROBOT_RADII, ROBOT_RADII);
  private static final double PREDICTION_DT = 0.2;

  @Getter private double timestamp;
  private Pose2d pose;
  private double vx;
  private double vy;
  public boolean isVisible;

  private final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.08, 0.02);
  private final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.08, 0.02);

  public ForeignRobot(double timestamp, Pose2d pose) {
    this.timestamp = timestamp;
    this.pose = pose;
    this.vx = 0;
    this.vy = 0;
    this.isVisible = true;
  }

  public double getSquaredDistance(Translation2d other) {
    return pose.getTranslation().getSquaredDistance(other);
  }

  public void updatePose(Pose2d newPose, double newTimestamp) {
    double deltaTime = newTimestamp - timestamp;

    this.vx = vxFilter.calculate((newPose.getX() - pose.getX()) / deltaTime);
    this.vy = vyFilter.calculate((newPose.getY() - pose.getY()) / deltaTime);

    this.timestamp = newTimestamp;
    this.pose = newPose;
  }

  public Pair<Translation2d, Translation2d> getPredictedCorners() {
    Translation2d predictedTranslation =
        new Translation2d(pose.getX() + vx * PREDICTION_DT, pose.getY() + vy * PREDICTION_DT);
    Logger.recordOutput(
        "Vision/ForeignRobotPosesPredicted", new Pose2d(predictedTranslation, pose.getRotation()));
    return Pair.of(
        predictedTranslation.plus(CORNER_OFFSET), predictedTranslation.minus(CORNER_OFFSET));
  }
}
