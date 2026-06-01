package frc.robot.util;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class MultiGoalADStar implements Pathfinder {
  private static final int MAX_GOALS = 12;

  private final List<LocalADStar> planners = new ArrayList<>();
  private final List<Translation2d> goalPositions = new ArrayList<>();

  private Translation2d lastStartPosition = Translation2d.kZero;
  private List<Pair<Translation2d, Translation2d>> lastObstacles = new ArrayList<>();
  private boolean lockGoals = false;

  public MultiGoalADStar() {
    for (int i = 0; i < MAX_GOALS; i++) {
      planners.add(new LocalADStar());
    }
  }

  public synchronized void setGoalPositions(List<Translation2d> goals) {
    lockGoals = false;

    goalPositions.clear();
    int count = Math.min(goals.size(), MAX_GOALS);

    for (int i = 0; i < count; i++) {
      Translation2d goal = goals.get(i);
      goalPositions.add(goal);

      LocalADStar planner = planners.get(i);
      planner.setStartPosition(lastStartPosition);
      if (!lastObstacles.isEmpty()) {
        planner.setDynamicObstacles(lastObstacles, lastStartPosition);
      }
      planner.setGoalPosition(goal);
    }

    lockGoals = true;
  }

  @Override
  public synchronized void setGoalPosition(Translation2d goalPosition) {
    if (!lockGoals) {
      setGoalPositions(List.of(goalPosition));
      lockGoals = false;
    }
  }

  @Override
  public synchronized void setStartPosition(Translation2d startPosition) {
    lastStartPosition = startPosition;
    for (int i = 0; i < goalPositions.size(); i++) {
      planners.get(i).setStartPosition(startPosition);
    }
  }

  @Override
  public synchronized void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    lastObstacles = obs;
    lastObstacles = obs;

    lastStartPosition = currentRobotPos; 

    for (int i = 0; i < goalPositions.size(); i++) {
        LocalADStar planner = planners.get(i);
        
        planner.setStartPosition(currentRobotPos);
        planner.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  @Override
  public synchronized boolean isNewPathAvailable() {
    for (int i = 0; i < goalPositions.size(); i++) {
      if (planners.get(i).isNewPathAvailable()) {
        return true;
      }
    }
    return false;
  }

  @Override
  public synchronized PathPlannerPath getCurrentPath(
      PathConstraints constraints, GoalEndState goalEndState) {

    PathPlannerPath bestPath = null;
    double bestLength = Double.POSITIVE_INFINITY;

    for (int i = 0; i < goalPositions.size(); i++) {
      PathPlannerPath candidate = planners.get(i).getCurrentPath(constraints, goalEndState);
      if (candidate == null) {
        continue;
      }

      double length = pathLength(candidate);
      if (length < bestLength) {
        bestLength = length;
        bestPath = candidate;
      }
    }

    return bestPath;
  }

  private static double pathLength(PathPlannerPath path) {
    List<Waypoint> waypoints = path.getWaypoints();
    if (waypoints == null || waypoints.size() < 2) {
      return Double.POSITIVE_INFINITY;
    }

    double total = 0.0;
    for (int i = 1; i < waypoints.size(); i++) {
      Translation2d prev = waypoints.get(i - 1).anchor();
      Translation2d curr = waypoints.get(i).anchor();
      total += prev.getDistance(curr);
    }
    return total;
  }

  public synchronized void unlock() {
    this.lockGoals = false;
  }
}
