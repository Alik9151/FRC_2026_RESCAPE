package frc.robot.util.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Optional;

public class Reef {

  private Pole[] poles;

  public Reef(int[] tags) {
    poles = new Pole[tags.length * 3];
    for (int i = 0; i < tags.length; i++) {
      addBranchesFromTag(getTagPose2d(tags[i]), i);
    }
  }

  public Pose2d getTagPose2d(int tagId) {
    Optional<Pose3d> tagPose3d = VisionConstants.APRIL_TAG_LAYOUT.getTagPose(tagId);
    return tagPose3d.get().toPose2d();
  }

  private void addBranchesFromTag(Pose2d tag, int tagNum) {
    poles[tagNum * 3] =
        new Pole(tag.getTranslation().plus(ReefConstants.leftOffset.rotateBy(tag.getRotation())));
    poles[tagNum * 3 + 1] = new Pole(tag.getTranslation());

    poles[tagNum * 3 + 2] =
        new Pole(tag.getTranslation().plus(ReefConstants.rightOffset.rotateBy(tag.getRotation())));
  }

  public Pole getBestPole(Translation2d robotPose) {
    double bestDistance = 0;
    Pole bestPole = null;

    for (Pole pole : poles) {
      int curLevel = pole.getMaxLevel();
      if (curLevel > bestPole.getMaxLevel()) {
        bestPole = pole;
        bestDistance = robotPose.getDistance(pole.getTranslation2d());
      } else if (curLevel == bestPole.getMaxLevel()) {
        double currentDistance = robotPose.getSquaredDistance(pole.getTranslation2d());
        if (currentDistance < bestDistance) {
          bestPole = pole;
          bestDistance = currentDistance;
        }
      }
    }

    return bestPole;
  }
}
