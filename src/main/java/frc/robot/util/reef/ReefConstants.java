package frc.robot.util.reef;

import edu.wpi.first.math.geometry.Translation2d;

public class ReefConstants {
  private static final double adjustX = 0.6;
  private static final double adjustY = -0.3209;

  public static final Translation2d LEFT_OFFSET = new Translation2d(adjustX, adjustY);
  public static final Translation2d RIGHT_OFFSET = new Translation2d(adjustX, -adjustY);

  public static final int[] BLUE_APRIL_TAGS = {17, 18, 19, 20, 21, 22};
  public static final int[] RED_APRIL_TAGS = {};
}
