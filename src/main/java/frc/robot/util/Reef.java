package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.*;

public class Reef {
  private final Pose2d[] polePoses;
  private final Map<Pose2d, Integer> poleIndices;
  private final boolean[][] open;
  private final int[] openSpots;

  public Reef(int[] tags) {
    polePoses = new Pose2d[tags.length * 2];
    poleIndices = new HashMap<>(polePoses.length);
    openSpots = new int[3];

    for (int i = 0; i < tags.length; i++) {
      addBranchesFromTag(getTagPose2d(tags[i]), i);
    }

    for (int i = 0; i < polePoses.length; i++) {
      poleIndices.put(polePoses[i], i);
    }

    open = new boolean[3][polePoses.length];

    for (int level = 0; level < 3; level++) {
      openSpots[level] = polePoses.length;
      for (int pole = 0; pole < polePoses.length; pole++) {
        open[level][pole] = true;
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
    for (boolean poleAvailable : open[levelIndex]) {
      if (poleAvailable) {
        count++;
      }
    }

    List<Pose2d> levelPoles = new ArrayList<>(count);

    for (int i = 0; i < polePoses.length; i++) {
      if (open[levelIndex][i]) {
        levelPoles.add(polePoses[i]);
      }
    }

    return levelPoles;
  }

  public Pose2d[] getPoses() {
    return polePoses;
  }

  public void updatePole(int level, Pose2d pole) {
    if (pole == null) return;
    Integer poleIndex = poleIndices.get(pole);
    if (poleIndex == null) return;

    if (open[level - 2][poleIndex]) {
      openSpots[level - 2]--;
    } else {
      openSpots[level - 2]++;
    }
    open[level - 2][poleIndex] = !open[level - 2][poleIndex];
  }

  public int getLevel() { // Finish this
    if (openSpots[2] > 8) return 4;
    if (openSpots[1] > 8) return 3;
    if (openSpots[0] > 8) return 2;
    if (openSpots[2] > 0) return 4;
    if (openSpots[1] > 0) return 3;
    if (openSpots[1] > 0) return 2;
    return 1;
  }
}
