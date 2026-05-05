package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class SuperstructureSim extends SubsystemBase {
  private final Intake intake;
  private final Elevator elevator;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  private final IntakeSimulation intakeSimulation;

  public SuperstructureSim(
      Intake intake,
      Elevator elevator,
      SwerveDriveSimulation swerveDriveSimulation,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.intake = intake;
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
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.FRONT,
            // The intake can hold up to 1 Coral
            1);
  }

  @Override
  public void simulationPeriodic() {
    if (intake.getVelocityRPS() > 38.0){
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
    
  }

  public void dropFuel(){
    ReefscapeCoralOnFly coralOnFly = new ReefscapeCoralOnFly(swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
    new Translation2d(0.2, -.2 + (Math.random() * (.4))),
    chassisSpeeds.get(),
    swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
    Meters.of(.45), //change to elevator state/height later
    MetersPerSecond.of(1),
    Degrees.of(65));

    coralOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();

    SimulatedArena.getInstance().addGamePieceProjectile(coralOnFly);
  }
  
}
