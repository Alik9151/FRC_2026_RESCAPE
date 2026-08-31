package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ForeignRobot {
  private static final double ROBOT_RADII = Units.inchesToMeters(16 * 2);
  private static final Translation2d CORNER_OFFSET = new Translation2d(ROBOT_RADII, ROBOT_RADII);
  private static final double PREDICTION_DT = 0.2;

  private static final double Q_POS = 0.01; // trust in our model
  private static final double Q_VEL = 0.1; // allows v to chang
  private static final double R_MEAS = 0.05; // trust in vision data

  @Getter private double timestamp;
  private Pose2d pose;
  private double vx;
  private double vy;
  public boolean isVisible;

  private final AxisKalmanFilter xKF;
  private final AxisKalmanFilter yKF;

  public ForeignRobot(double timestamp, Pose2d pose) {
    this.timestamp = timestamp;
    this.pose = pose;
    this.vx = 0;
    this.vy = 0;
    this.isVisible = true;

    this.xKF = new AxisKalmanFilter(Q_POS, Q_VEL, R_MEAS, pose.getTranslation().getX());
    this.yKF = new AxisKalmanFilter(Q_POS, Q_VEL, R_MEAS, pose.getTranslation().getY());
  }

  public double getSquaredDistance(Translation2d other) {
    return pose.getTranslation().getSquaredDistance(other);
  }

  public void updatePose(Pose2d newPose, double newTimestamp) {
    double dt = newTimestamp - timestamp;

    xKF.predict(dt);
    yKF.predict(dt);

    xKF.update(newPose.getTranslation().getX());
    yKF.update(newPose.getTranslation().getY());

    this.timestamp = newTimestamp;
    this.vx = xKF.getVelocity();
    this.vy = yKF.getVelocity();
    this.pose = newPose;
  }

  public Pair<Translation2d, Translation2d> getPredictedCorners() {
    Translation2d predictedTranslation =
        new Translation2d(
            pose.getTranslation().getX() + vx * PREDICTION_DT,
            pose.getTranslation().getY() + vy * PREDICTION_DT);

    Logger.recordOutput(
        "Vision/ForeignRobotPosesPredicted", new Pose2d(predictedTranslation, Rotation2d.kZero));

    return Pair.of(
        predictedTranslation.plus(CORNER_OFFSET), predictedTranslation.minus(CORNER_OFFSET));
  }

  private static class AxisKalmanFilter {
    private double p, v;
    private double pVar = 1.0, vVar = 1.0, pvCov = 0.0;
    private final double qPos, qVel, rMeas;

    public AxisKalmanFilter(double qPos, double qVel, double rMeas, double initialPos) {
      this.qPos = qPos;
      this.qVel = qVel;
      this.rMeas = rMeas;
      this.p = initialPos;
      this.v = 0.0;
    }

    public void predict(double dt) {
      p += v * dt;

      pVar += dt * dt * vVar + 2 * dt * pvCov + qPos;
      pvCov += dt * vVar;
      vVar += qVel;
    }

    public void update(double measP) {
      double s = pVar + rMeas;
      double kP = pVar / s;
      double kV = pvCov / s;

      double y = measP - p;

      p += kP * y;
      v += kV * y;

      double pVarNew = (1 - kP) * pVar;
      double pvCovNew = (1 - kP) * pvCov;
      double vVarNew = -kV * pvCov + vVar;

      pVar = pVarNew;
      pvCov = pvCovNew;
      vVar = vVarNew;
    }

    public double getPosition() {
      return p;
    }

    public double getVelocity() {
      return v;
    }
  }
}
