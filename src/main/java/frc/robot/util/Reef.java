package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Reef {
  private final Pose2d[] polePoses;
  private final Map<Pose2d, Integer> poleIndexes;
  private final boolean[][] available;

  public Reef(int[] tags) {
    polePoses = new Pose2d[tags.length * 2];
    poleIndexes = new HashMap<>(polePoses.length);

    for (int i = 0; i < tags.length; i++) {
      addBranchesFromTag(getTagPose2d(tags[i]), i);
    }

    for (int i = 0; i < polePoses.length; i++) {
      poleIndexes.put(polePoses[i], i);
    }

    available = new boolean[3][polePoses.length];

    for (int level = 0; level < 3; level++) {
      for (int pole = 0; pole < polePoses.length; pole++) {
        available[level][pole] = true;
      }
    }
  }

  private Pose2d getTagPose2d(int tagId) {
    return VisionConstants.APRIL_TAG_LAYOUT.getTagPose(tagId).orElseThrow().toPose2d();
  }

  private void addBranchesFromTag(Pose2d tag, int tagNum) {
    Rotation2d poleRotation = tag.getRotation().plus(Rotation2d.k180deg);

    polePoses[tagNum * 2] =
        new Pose2d(
            tag.getTranslation()
                .plus(Constants.FieldConstants.LEFT_REEF_OFFSET.rotateBy(tag.getRotation())),
            poleRotation);

    polePoses[tagNum * 2 + 1] =
        new Pose2d(
            tag.getTranslation()
                .plus(Constants.FieldConstants.RIGHT_REEF_OFFSET.rotateBy(tag.getRotation())),
            poleRotation);
  }

  public List<Pose2d> getPoles(int level) {
    int levelIndex = level - 2;

    int count = 0;
    for (boolean poleAvailable : available[levelIndex]) {
      if (poleAvailable) {
        count++;
      }
    }

    List<Pose2d> levelPoles = new ArrayList<>(count);

    for (int i = 0; i < polePoses.length; i++) {
      if (available[levelIndex][i]) {
        levelPoles.add(polePoses[i]);
      }
    }

    return levelPoles;
  }

  public Pose2d[] getPoses() {
    return polePoses;
  }

  public void updatePole(int level, Pose2d pole) {
    Integer poleIndex = poleIndexes.get(pole);

    if (poleIndex == null) {
      return;
    }

    available[level - 2][poleIndex] = !available[level - 2][poleIndex];
  }
}
