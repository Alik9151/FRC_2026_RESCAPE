package frc.robot.util.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide;
import org.littletonrobotics.junction.Logger;

public class SuperstructureSim {
  private final Elevator elevator;
  private final Intake intake;
  private final Outtake outtake;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  private final IntakeSimulation intakeSimulation;

  private double stage1Height;
  private double stage2Height;

  private Translation3d outtakeTranslation;
  private Rotation3d outtakeRotation;
  private Pose3d localCoral;

  // Define the length of the outtake arm from pivot to the center of the held Coral
  private static final double OUTTAKE_ARM_LENGTH_METERS = 0.35;

  public SuperstructureSim(
      Elevator elevator,
      Intake intake,
      Outtake outtake,
      SwerveDriveSimulation swerveDriveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.elevator = elevator;
    this.intake = intake;
    this.outtake = outtake;
    this.swerveDriveSimulation = swerveDriveSimulation;
    this.chassisSpeeds = chassisSpeeds;

    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            swerveDriveSimulation,
            Meters.of(0.7),
            Meters.of(0.2),
            IntakeSimulation.IntakeSide.BACK,
            1);
  }

  public void simulationPeriodic() {
    if (intake.getVelocityRPS() > 70.0) intakeSimulation.startIntake();
    else intakeSimulation.stopIntake();

    double carriageHeight = ElevatorConstants.radiansToMeters(elevator.getPositionRad());

    stage1Height = carriageHeight / 2.0;
    stage2Height = carriageHeight;

    outtakeTranslation = new Translation3d(0.2, 0.0, 0.55 + stage2Height);
    outtakeRotation = new Rotation3d(0, Math.toRadians(-outtake.getPositionDeg()), 0);

    Logger.recordOutput(
        "FieldSimulation/RobotComponentPositions",
        new Pose3d(0.0, 0.0, stage1Height, Rotation3d.kZero), // stage 1
        new Pose3d(0.0, 0.0, stage2Height, Rotation3d.kZero), // stage 2
        new Pose3d(
            -0.305,
            0,
            0.23,
            new Rotation3d(0, Math.toRadians(42.5 - intake.getPositionDeg()), 0)), // intake
        new Pose3d(outtakeTranslation, outtakeRotation)); // outtake

    if (isLoaded()) {
      localCoral = getCoralRobotRelativePose();

      Pose2d simDrivePose = swerveDriveSimulation.getSimulatedDriveTrainPose();

      Translation3d globalTranslationOffset =
          localCoral
              .getTranslation()
              .rotateBy(new Rotation3d(0, 0, simDrivePose.getRotation().getRadians()));

      Translation3d globalCoralTranslation =
          new Translation3d(simDrivePose.getX(), simDrivePose.getY(), 0.0)
              .plus(globalTranslationOffset);

      Rotation3d globalCoralRotation =
          new Rotation3d(
              localCoral.getRotation().getX(),
              localCoral.getRotation().getY(),
              localCoral.getRotation().getZ() + simDrivePose.getRotation().getRadians());

      Pose3d globalCoralPose = new Pose3d(globalCoralTranslation, globalCoralRotation);

      Logger.recordOutput("FieldSimulation/CoralInBot", globalCoralPose);
    } else {
      Logger.recordOutput("FieldSimulation/CoralInBot", new Pose3d());
    }

    if (outtake.getVelocityRPS() > 50.0) scoreFuel();
  }

  private Pose3d getCoralRobotRelativePose() {
    Translation3d rotatedCoralOffset =
        new Translation3d(0.0, 0.0, -OUTTAKE_ARM_LENGTH_METERS).rotateBy(outtakeRotation);
    Translation3d coralInRobotSpace = outtakeTranslation.plus(rotatedCoralOffset);

    return new Pose3d(coralInRobotSpace, outtakeRotation);
  }

  public boolean isLoaded() {
    return intakeSimulation.getGamePiecesAmount() >= 1;
  }

  public void scoreFuel() {
    if (!intakeSimulation.obtainGamePieceFromIntake()) {
      return;
    }

    Translation3d localCoralTranslation = localCoral.getTranslation();

    ReefscapeCoralOnFly coralOnFly =
        new ReefscapeCoralOnFly(
            swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            new Translation2d(localCoralTranslation.getX(), localCoralTranslation.getY()),
            chassisSpeeds.get(),
            swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
            Meters.of(localCoralTranslation.getZ()),
            MetersPerSecond.of(-1),
            Degrees.of(outtake.getPositionDeg()));

    coralOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }

  public void loadFuel(CoralStationsSide side) {
    ReefscapeCoralOnFly coralOnFly =
        ReefscapeCoralOnFly.DropFromCoralStation(side, DriverStation.getAlliance().get(), true);
    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }
}
