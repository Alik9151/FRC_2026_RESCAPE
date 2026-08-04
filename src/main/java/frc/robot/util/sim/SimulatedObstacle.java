package frc.robot.util.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.RobotUtil;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class SimulatedObstacle {
  private static final double MAX_LINEAR_SPEED = 4; // m/s
  private static final double MAX_ANGULAR_SPEED = 5; // rad/s

  private static final StructArrayPublisher<Translation2d> publisher =
      NetworkTableInstance.getDefault()
          .getTable(PhotonCamera.kTableName)
          .getSubTable(VisionConstants.CAMERA_0_NAME)
          .getStructArrayTopic("foreignRobotPoses", Translation2d.struct)
          .publish();
  private static Supplier<Pose2d> robotPoseSupplier;
  private static final ArrayList<SimulatedObstacle> obstacles = new ArrayList<>(2);
  private static Pose2d[] poses = new Pose2d[0];

  private final SwerveDriveSimulation driveSimulation;
  private Pose2d pose;
  private ChassisSpeeds speeds;
  private double lastTime = -1;

  public static SimulatedObstacle[] createObstacles(int amount, Supplier<Pose2d> poseSupplier) {
    robotPoseSupplier = poseSupplier;
    SimulatedObstacle[] newObstacles = new SimulatedObstacle[amount];
    for (int i = 0; i < newObstacles.length; i++) {
      SimulatedObstacle newObstacle = new SimulatedObstacle();
      newObstacles[i] = newObstacle;
      obstacles.add(newObstacle);
    }
    poses = new Pose2d[obstacles.size()];
    return newObstacles;
  }

  public static void periodic() {
    for (int i = 0; i < poses.length; i++) {
      poses[i] = obstacles.get(i).pose;
    }
    Logger.recordOutput("SimulatedObstacles/ActualPoses", poses);

    Translation2d[] translations = new Translation2d[poses.length];
    for (int i = 0; i < translations.length; i++) {
      translations[i] = globalPoseToCameraRelative(poses[i]);
    }

    publisher.set(translations);
  }

  private static Translation2d globalPoseToCameraRelative(Pose2d pose) {
    Pose2d robotPose = robotPoseSupplier.get();
    return pose.getTranslation()
        .minus(robotPose.getTranslation())
        .rotateBy(robotPose.getRotation().unaryMinus())
        .minus(VisionConstants.robotToCamera0.getTranslation().toTranslation2d())
        .rotateBy(VisionConstants.robotToCamera0.getRotation().toRotation2d().unaryMinus());
  }

  private SimulatedObstacle() {
    this(new Pose2d(3, 3, Rotation2d.kZero));
  }

  private SimulatedObstacle(Pose2d initialPose) {
    this.pose = initialPose;
    driveSimulation =
        new SwerveDriveSimulation(
            Drive.getMapleSimConfig().withGyro(() -> new GyroSimulation(0, 0)), initialPose);
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
  }

  public Command move(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
            () -> {
              if (lastTime < 0) {
                lastTime = Timer.getTimestamp();
                return;
              }
              double currentTime = Timer.getTimestamp();
              double dt = currentTime - lastTime;
              lastTime = currentTime;

              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSupplier.getAsDouble() * MAX_LINEAR_SPEED,
                      ySupplier.getAsDouble() * MAX_LINEAR_SPEED,
                      omegaSupplier.getAsDouble() * MAX_ANGULAR_SPEED,
                      RobotUtil.isRedAlliance()
                          ? pose.getRotation().plus(Rotation2d.kPi)
                          : pose.getRotation());
              pose =
                  pose.transformBy(
                      new Transform2d(
                          speeds.vxMetersPerSecond * dt,
                          speeds.vyMetersPerSecond * dt,
                          Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
              driveSimulation.setSimulationWorldPose(pose);
            })
        .beforeStarting(() -> lastTime = -1);
  }
}
