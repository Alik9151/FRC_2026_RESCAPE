package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ForeignRobot {
  private static final double ROBOT_RADII = Units.inchesToMeters(17.5 * 2);
  private static final Translation2d CORNER_OFFSET = new Translation2d(ROBOT_RADII, ROBOT_RADII);
  private static final double PREDICTION_DT = 0.2;

  private static int robotCount = 0;

  private final int id;
  @Getter private double timestamp;
  private Translation2d translation;
  private double vx;
  private double vy;
  public boolean isVisible;

  private final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.08, 0.02);
  private final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.08, 0.02);

  public ForeignRobot(double timestamp, Translation2d translation) {
    this.id = robotCount++;
    this.timestamp = timestamp;
    this.translation = translation;
    this.vx = 0;
    this.vy = 0;
    this.isVisible = true;
  }

  public double getSquaredDistance(Translation2d other) {
    return translation.getSquaredDistance(other);
  }

  public void updateTranslation(Translation2d newTranslation, double newTimestamp) {
    double deltaTime = newTimestamp - timestamp;

    this.vx = vxFilter.calculate((newTranslation.getX() - translation.getX()) / deltaTime);
    this.vy = vyFilter.calculate((newTranslation.getY() - translation.getY()) / deltaTime);

    this.timestamp = newTimestamp;
    this.translation = newTranslation;
  }

  public Pair<Translation2d, Translation2d> getPredictedCorners() {
    Translation2d predictedTranslation =
        new Translation2d(
            translation.getX() + vx * PREDICTION_DT, translation.getY() + vy * PREDICTION_DT);
    Logger.recordOutput(
        "Vision/ForeignRobotPosesPredicted", new Pose2d(predictedTranslation, Rotation2d.kZero));
    return Pair.of(
        predictedTranslation.plus(CORNER_OFFSET), predictedTranslation.minus(CORNER_OFFSET));
  }
}
