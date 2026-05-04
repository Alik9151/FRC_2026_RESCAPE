// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVER_DEADBAND = 0.1;
    public static final double OPERATOR_DEADBAND = 0.1;
  }

  public static final class CANConstants {
    public static final CANBus SUPERSTRUCTURE_CAN_BUS = new CANBus("Superstructure");

    public static final int ELEVATOR_LEADER = 0;
    public static final int ELEVATOR_FOLLOWER = 0;

    public static final int INTAKE = 0;
    public static final int OUTTAKE = 0;
  }

  public static final class FieldConstants {
    public static final Distance FIELD_LENGTH =
        Meters.of(VisionConstants.APRIL_TAG_LAYOUT.getFieldLength());
    public static final Distance FIELD_WIDTH =
        Meters.of(VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth());

    public static final Pose2d LOADING_STATION_LEFT =
        VisionConstants.APRIL_TAG_LAYOUT.getTagPose(13).get().toPose2d();
    public static final Pose2d LOADING_STATION_RIGHT =
        VisionConstants.APRIL_TAG_LAYOUT.getTagPose(12).get().toPose2d();

    private static final double REEF_ADJUST_X = 0.6;
    private static final double REEF_ADJUST_Y = -0.3209;

    public static final Translation2d LEFT_REEF_OFFSET =
        new Translation2d(REEF_ADJUST_X, REEF_ADJUST_Y);
    public static final Translation2d RIGHT_REEF_OFFSET =
        new Translation2d(REEF_ADJUST_X, -REEF_ADJUST_Y);

    public static final int[] BLUE_REEF_APRIL_TAGS = {17, 18, 19, 20, 21, 22};
    public static final int[] RED_REEF_APRIL_TAGS = {};
  }
}
