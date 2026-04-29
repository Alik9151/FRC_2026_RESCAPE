// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.currentMode;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.BetterAutoChooser;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RobotUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // subsystems
  private final Drive drive;
  private final Vision vision;

  // controllers
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  // dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Simulated things
  private SwerveDriveSimulation driveSimulation;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {});
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_0_NAME, VisionConstants.robotToCamera0),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_1_NAME, VisionConstants.robotToCamera1),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_2_NAME, VisionConstants.robotToCamera2),
        //                new VisionIOPhotonVision(
        //                    VisionConstants.CAMERA_3_NAME, VisionConstants.robotToCamera3));
        break;
      case SIM:
        SimulatedArena.overrideInstance(new Arena2025Reefscape());
        SimulatedArena.getInstance().resetFieldForAuto();
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()) {},
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_0_NAME,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_1_NAME,
                    VisionConstants.robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose));
        //                new VisionIO() {});
        break;
      default:
        // replay
        SimulatedArena.overrideInstance(new Arena2025Reefscape());
        SimulatedArena.getInstance().resetFieldForAuto();
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision =
            new Vision(
                drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
    }

    PhoenixUtil.startTelemetry();

    // Configure the trigger bindings
    configureBindings();

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser", BetterAutoChooser.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set the default auto (do nothing)
    autoChooser.addDefaultOption("Do Nothing", Commands.none());

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    RobotUtil.setDriverController(driverController);
    RobotUtil.setOperatorController(operatorController);

    // Default command, normal field-relative drive
    Command defaultDriveCommand =
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX());
    // Lock wheels to X pattern
    Command lockWheels = Commands.startEnd(drive::stopWithX, () -> {}, drive);
    // Reset gyro to 0°
    Command zeroGyro = Commands.runOnce(() -> drive.zeroGyro(true), drive).ignoringDisable(true);

    drive.setDefaultCommand(defaultDriveCommand);

    if (Constants.currentMode == Constants.Mode.SIM) {
      // CommandGenericHID keyboard = new CommandGenericHID(2);
      // keyboard.button(1).whileTrue(holdIntake);
    }

    if (DriverStation.isTest()) {
      // single controller for testing
      driverController.x().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);
    } else {
      // driver controls
      driverController.leftBumper().whileTrue(lockWheels);
      driverController.povLeft().onTrue(zeroGyro);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode == Constants.Mode.REAL) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Pose3d[] CoralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    Pose3d[] AlgaePoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");

    Pose2d simPose = driveSimulation.getSimulatedDriveTrainPose();

    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/RobotPosition", simPose);
    // to set up the model
    Logger.recordOutput("FieldSimulation/CoralPositions", CoralPoses);
    Logger.recordOutput("FieldSimulation/AlgaePositions", AlgaePoses);
  }
}
