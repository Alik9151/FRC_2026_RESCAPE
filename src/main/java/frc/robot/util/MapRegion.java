package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.Path2D;
import org.littletonrobotics.junction.Logger;

public class MapRegion {
  String name;
  Path2D shapeOutline;
  Translation2d[] points;

  public MapRegion(String name, Translation2d[] points) {
    this.name = name;
    this.points = points;
    generateOutline();
    Logger.recordOutput("Zones/" + name, points);
  }

  public void generateOutline() {
    shapeOutline = new Path2D.Double(Path2D.WIND_EVEN_ODD, points.length);
    shapeOutline.moveTo(points[0].getX(), points[0].getY());

    for (int i = 1; i < points.length; i++) {
      shapeOutline.lineTo(points[0].getX(), points[0].getY());
    }
    shapeOutline.closePath();
  }

  public boolean contains(Pose2d pose) {
    return shapeOutline.contains(pose.getX(), pose.getY());
  }

  public String name() {
    return name;
  }
}
