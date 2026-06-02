package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Reef {
  private final Pose2d[] polePoses;
  private final List<List<Integer>> levels;

  public Reef(int[] tags) {
    polePoses = new Pose2d[tags.length * 2];

    for (int i = 0; i < tags.length; i++) {
      addBranchesFromTag(getTagPose2d(tags[i]), i);
    }

    levels = new ArrayList<>(3);
    for (int level = 0; level < 3; level++) {
      ArrayList<Integer> currentLevel = new ArrayList<>(polePoses.length);

      for (int pole = 0; pole < polePoses.length; pole++) {
        currentLevel.add(pole);
      }

      this.levels.add(currentLevel);
    }
  }

  public Pose2d getTagPose2d(int tagId) {
    Optional<Pose3d> tagPose3d = VisionConstants.APRIL_TAG_LAYOUT.getTagPose(tagId);
    return tagPose3d.get().toPose2d();
  }

  private void addBranchesFromTag(Pose2d tag, int tagNum) {
    polePoses[tagNum * 2] =
        new Pose2d(
            tag.getTranslation()
                .plus(Constants.FieldConstants.LEFT_REEF_OFFSET.rotateBy(tag.getRotation())),
            tag.getRotation().plus(Rotation2d.k180deg));
    polePoses[tagNum * 2 + 1] =
        new Pose2d(
            tag.getTranslation()
                .plus(Constants.FieldConstants.RIGHT_REEF_OFFSET.rotateBy(tag.getRotation())),
            tag.getRotation().plus(Rotation2d.k180deg));
  }

  public List<Pose2d> getPoles(int level) {
    List<Pose2d> levelPoles = new ArrayList<>();
    for (int i : levels.get(level - 2)) {
      levelPoles.add(polePoses[i]);
    }
    return levelPoles;
  }

  public Pose2d[] getPoses() {
    return polePoses;
  }

  public void updatePole(int level, Pose2d pole) {
    int poleNum = findIndex(pole);
    List<Integer> levelList = levels.get(level - 2);
    int index = levelList.indexOf(poleNum);
    if (index == -1) {
      levelList.add(poleNum);
    } else {
      System.out.println("REMOVING");
      levelList.remove(index);
    }
  }

  private int findIndex(Pose2d target) {
    for (int i = 0; i < polePoses.length; i++) {
      if (polePoses[i].equals(target)) {
        return i;
      }
    }
    return -1;
  }
}
