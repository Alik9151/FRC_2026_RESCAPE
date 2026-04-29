package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(
          AprilTagFields
              .k2026RebuiltAndymark); // Chesapeake = AndyMark, FIRST Championship = Welded

  // Camera names, must match names configured on coprocessor
  public static final String CAMERA_0_NAME = "Camera_Left";
  public static final String CAMERA_1_NAME = "Camera_Right";
  public static final String CAMERA_2_NAME = "Camera_Right";
  public static final String CAMERA_3_NAME = "Camera_Right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(-12.5), // x
          Units.inchesToMeters(16), // y
          Units.inchesToMeters(7.5), // z
          new Rotation3d(
              Units.degreesToRadians(2.0), // roll
              Units.degreesToRadians(-6.0), // pitch
              Units.degreesToRadians(150.0))); // yaw
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(-12.5), // x
          Units.inchesToMeters(-16), // y
          Units.inchesToMeters(7.5), // z
          new Rotation3d(
              Units.degreesToRadians(0.0), // roll
              Units.degreesToRadians(-1.6), // pitch
              Units.degreesToRadians(-150.0))); // yaw
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(-12.5), // x
          Units.inchesToMeters(16), // y
          Units.inchesToMeters(7.5), // z
          new Rotation3d(
              Units.degreesToRadians(2.0), // roll
              Units.degreesToRadians(-6.0), // pitch
              Units.degreesToRadians(150.0))); // yaw
  public static Transform3d robotToCamera3 =
      new Transform3d(
          Units.inchesToMeters(-12.5), // x
          Units.inchesToMeters(-16), // y
          Units.inchesToMeters(7.5), // z
          new Rotation3d(
              Units.degreesToRadians(0.0), // roll
              Units.degreesToRadians(-1.6), // pitch
              Units.degreesToRadians(-150.0))); // yaw

  // Basic filtering thresholds
  public static final double MAX_AMBIGUITY = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

  // Standard deviation baselines, for 1-meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.6; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {1.0, 1.0, 1.0, 1.0};

  // Multipliers to apply for MegaTag 2 observations (Limelight only)
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
