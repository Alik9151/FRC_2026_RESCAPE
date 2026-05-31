package frc.robot.util.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RobotUtil;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.photonvision.PhotonCamera;

public class SimulatedObstacle {
  private static final double MAX_LINEAR_SPEED = 4; // m/s
  private static final double MAX_ANGULAR_SPEED = 5; // rad/s

  private static final StructArrayPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault()
          .getTable(PhotonCamera.kTableName)
          .getStructArrayTopic("foreignRobotPoses", Pose2d.struct)
          .publish();
  private static final ArrayList<SimulatedObstacle> obstacles = new ArrayList<>();
  private static Pose2d[] poses = new Pose2d[0];

  private static DriveTrainSimulationConfig mapleSimConfig = null;

  public static DriveTrainSimulationConfig getMapleSimConfig() {
    if (mapleSimConfig != null) return mapleSimConfig;

    return mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(Drive.getModuleTranslations())
            .withGyro(() -> new GyroSimulation(0, 0));
  }

  private final SwerveDriveSimulation driveSimulation;
  private Pose2d pose;
  private ChassisSpeeds speeds;
  private double lastTime = -1;

  public static SimulatedObstacle[] createObstacles(int amount) {
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
    publisher.set(poses);
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
