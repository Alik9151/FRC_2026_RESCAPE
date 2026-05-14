package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.STAGE_1_MAX_HEIGHT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.function.Supplier;
import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide;
import org.littletonrobotics.junction.Logger;

public class SuperstructureSim {
  private final Elevator elevator;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  private final IntakeSimulation intakeSimulation;

  private double stage2Height;

  public SuperstructureSim(
      Elevator elevator,
      SwerveDriveSimulation swerveDriveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.elevator = elevator;
    this.swerveDriveSimulation = swerveDriveSimulation;
    this.chassisSpeeds = chassisSpeeds;

    intakeSimulation =
        new IntakeSimulation(
            // Specify the type of game pieces that the intake can collect
            "Coral",
            // Specify the drivetrain to which this intake is attached
            swerveDriveSimulation,
            // Width of the intake
            new Rectangle(0.7, 0.5),
            // The intake can hold up to 1 Coral
            1);
  }

  public void simulationPeriodic() {
    double carriageHeight = ElevatorConstants.radiansToMeters(elevator.getPositionRad());

    double stage1Height = MathUtil.clamp(carriageHeight / 2.0, 0.0, STAGE_1_MAX_HEIGHT);
    stage2Height = carriageHeight;

    // Logger.recordOutput("FieldSimulation/Tuning", new Pose3d(0.0, 0.0, 0.0, Rotation3d.kZero));
    Logger.recordOutput(
        "FieldSimulation/RobotComponentPositions",
        new Pose3d(0.0, 0.0, stage1Height, Rotation3d.kZero),
        new Pose3d(0.0, 0.0, stage2Height, Rotation3d.kZero),
        new Pose3d(-0.305, 0, 0.23, new Rotation3d(0, Math.toRadians(42.5), 0)),
        new Pose3d(0.2, 0.0, 0.55 + stage2Height, Rotation3d.kZero));

    if (isLoaded()) {
      Pose2d simDrivePose = swerveDriveSimulation.getSimulatedDriveTrainPose();
      Translation2d coralTranslation =
          simDrivePose
              .getTranslation()
              .plus(new Translation2d(0.25, 0).rotateBy(simDrivePose.getRotation()));
      Logger.recordOutput(
          "FieldSimulation/CoralInBot",
          new Pose3d(
              coralTranslation.getX(),
              coralTranslation.getY(),
              stage2Height + 0.7,
              new Rotation3d(simDrivePose.getRotation())));
    } else {
      Logger.recordOutput("FieldSimulation/CoralInBot", new Pose3d());
    }
  }

  public void startIntake() {
    intakeSimulation.startIntake();
  }

  public void stopIntake() {
    intakeSimulation.stopIntake();
  }

  public boolean isLoaded() {
    return intakeSimulation.getGamePiecesAmount() >= 1;
  }

  public void scoreFuel() {
    if (!intakeSimulation.obtainGamePieceFromIntake()) {
      return;
    }

    ReefscapeCoralOnFly coralOnFly =
        new ReefscapeCoralOnFly(
            swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            new Translation2d(.275, 0),
            chassisSpeeds.get(),
            swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
            Meters.of(stage2Height + 0.7), // change to elevator state/height later
            MetersPerSecond.of(0.75),
            Degrees.of(-20));

    coralOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }

  public void loadFuel(CoralStationsSide side) {
    ReefscapeCoralOnFly coralOnFly =
        ReefscapeCoralOnFly.DropFromCoralStation(side, DriverStation.getAlliance().get(), false);
    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }
}
