package frc.robot.util.reef;

import edu.wpi.first.math.geometry.Translation2d;

public class Pole {
  private Translation2d pose;
  private boolean[] levelFull;
  private int maxLevel;

  public Pole(Translation2d pose) {
    this.pose = pose;
    levelFull = new boolean[3];
    maxLevel = 4;
  }

  public void updateLevel(int level) {
    levelFull[level - 2] = !levelFull[level - 2];
    findMaxLevel();
  }

  private void findMaxLevel() {
    if (!levelFull[2]) maxLevel = 4;
    else if (!levelFull[1]) maxLevel = 3;
    else if (!levelFull[0]) maxLevel = 2;
    maxLevel = -1;
  }

  public int getMaxLevel() {
    return maxLevel;
  }

  public Translation2d getTranslation2d() {
    return pose;
  }
}
