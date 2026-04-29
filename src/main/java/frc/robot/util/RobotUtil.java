package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class contains methods that are used throughout the codebase and are not bound to one
 * subsystem or class.
 */
public final class RobotUtil {
  public static boolean isPoseEstimatorReady;
  private static CommandXboxController driverController;
  private static CommandXboxController operatorController;

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /** Instantiate the Xbox controllers. */
  public static void setDriverController(CommandXboxController driverControllerInstance) {
    driverController = driverControllerInstance;
  }

  public static void setOperatorController(CommandXboxController operatorControllerInstance) {
    operatorController = operatorControllerInstance;
  }

  /** Set the Rumble value for each Xbox controller. */
  public static void setDriverRumble(double leftRumble, double rightRumble) {
    driverController.setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
    driverController.setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
  }

  public static void setOperatorRumble(double leftRumble, double rightRumble) {
    operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
    operatorController.setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
  }
}
