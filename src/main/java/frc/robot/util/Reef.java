package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Reef {
  private final Pole[] poles;

  public Reef(int[] tags) {
    poles = new Pole[tags.length * 2];
    for (int i = 0; i < tags.length; i++) {
      addBranchesFromTag(getTagPose2d(tags[i]), i);
    }
  }

  public Pose2d getTagPose2d(int tagId) {
    Optional<Pose3d> tagPose3d = VisionConstants.APRIL_TAG_LAYOUT.getTagPose(tagId);
    return tagPose3d.get().toPose2d();
  }

  private void addBranchesFromTag(Pose2d tag, int tagNum) {
    poles[tagNum * 2] =
        new Pole(
            new Pose2d(
                tag.getTranslation()
                    .plus(Constants.FieldConstants.LEFT_REEF_OFFSET.rotateBy(tag.getRotation())),
                tag.getRotation().plus(Rotation2d.k180deg)));
    poles[tagNum * 2 + 1] =
        new Pole(
            new Pose2d(
                tag.getTranslation()
                    .plus(Constants.FieldConstants.RIGHT_REEF_OFFSET.rotateBy(tag.getRotation())),
                tag.getRotation().plus(Rotation2d.k180deg)));
  }

  public Pole getBestPole(Translation2d robotPose) {
    double bestDistance = 0;
    Pole bestPole = null;

    for (Pole pole : poles) {
      int curLevel = pole.getMaxLevel();
      if (bestPole == null || curLevel > bestPole.getMaxLevel()) {
        bestPole = pole;
        bestDistance = robotPose.getSquaredDistance(pole.getTranslation());
      } else if (curLevel == bestPole.getMaxLevel()) {
        double currentDistance = robotPose.getSquaredDistance(pole.getTranslation());
        if (currentDistance < bestDistance) {
          bestPole = pole;
          bestDistance = currentDistance;
        }
      }
    }

    return bestPole;
  }

  public Pose2d[] getPoses() {
    Pose2d[] polePoses = new Pose2d[poles.length];
    for (int i = 0; i < poles.length; i++) {
      polePoses[i] = poles[i].getPose();
    }
    return polePoses;
  }

  public static class Pole {
    @Getter private final Pose2d pose;
    @Getter private final Translation2d translation;
    private final boolean[] levels;
    @Getter private int maxLevel;

    private final String prefix = "FieldElements/Reef/Branch" + hashCode() + "/";

    public Pole(Pose2d pose) {
      this.pose = pose;
      this.translation = pose.getTranslation();
      levels = new boolean[3];
      maxLevel = 4;
      Logger.recordOutput(prefix + "Levels", levels);
      Logger.recordOutput(prefix + "MaxLevel", maxLevel);
    }

    public void updateLevel(int level) {
      levels[level - 2] = !levels[level - 2];
      findMaxLevel();
      Logger.recordOutput(prefix + "Levels", levels);
      Logger.recordOutput(prefix + "MaxLevel", maxLevel);
    }

    private void findMaxLevel() {
      if (!levels[2]) maxLevel = 4;
      else if (!levels[1]) maxLevel = 3;
      else if (!levels[0]) maxLevel = 2;
      else maxLevel = 0;
    }
  }
}
