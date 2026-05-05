package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide;

public class SuperstructureSim {
  private final Elevator elevator;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  private final IntakeSimulation intakeSimulation;

  public SuperstructureSim(
      Elevator elevator,
      SwerveDriveSimulation swerveDriveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.elevator = elevator;
    this.swerveDriveSimulation = swerveDriveSimulation;
    this.chassisSpeeds = chassisSpeeds;
    intakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            // Specify the type of game pieces that the intake can collect
            "Coral",
            // Specify the drivetrain to which this intake is attached
            swerveDriveSimulation,
            // Width of the intake
            Meters.of(0.7),
            // The intake is mounted on the front side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 1 Coral
            1);
  }

  public void simulationPeriodic() {
    // todo
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
            Translation2d.kZero,
            chassisSpeeds.get(),
            swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
            Meters.of(.45), // change to elevator state/height later
            MetersPerSecond.of(1),
            Degrees.of(65));

    coralOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }

  public void loadFuel(CoralStationsSide side) {
    ReefscapeCoralOnFly coralOnFly =
        ReefscapeCoralOnFly.DropFromCoralStation(side, DriverStation.getAlliance().get(), false);
    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }
}
