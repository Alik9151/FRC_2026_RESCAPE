package frc.robot.util.reef;

import edu.wpi.first.math.geometry.Translation2d;

public class ReefConstants {
  private static final double adjustX = 0.6;
  private static final double adjustY = -0.3209;

  public static final Translation2d leftOffset = new Translation2d(adjustX, adjustY);
  public static final Translation2d rightOffset = new Translation2d(adjustX, -adjustY);

  public static final int[] blueAprilTags = {17, 18, 19, 20, 21, 22};
}
