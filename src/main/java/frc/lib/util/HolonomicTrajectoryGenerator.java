// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class HolonomicTrajectoryGenerator {
    public static Trajectory generateTrajectory(List<Pose2d> waypoints, TrajectoryConfig config) {

        int numWaypoints = waypoints.size();
        Pose2d start = waypoints.get(0);
        Pose2d end = waypoints.get(numWaypoints-1);
        //angle of vector from start waypoint to end
        Rotation2d rot = new Rotation2d(Math.tan((start.getY() - end.getY()) / (start.getX() - end.getX())));
        //need to make end ang ang between last and 2nd to last
        Pose2d newStart = new Pose2d(start.getX(), start.getY(), rot);
        Pose2d newEnd = new Pose2d(end.getX(), end.getY(), rot);
        List<Translation2d> interiorWaypoints = new ArrayList<>();

        for(int i = 1; i < numWaypoints-1; i++) {
            var interior = waypoints.get(i);
            interiorWaypoints.add(new Translation2d(interior.getX(), interior.getY()));
        }

        Trajectory traj = TrajectoryGenerator.generateTrajectory(newStart, interiorWaypoints, newEnd, config);

        //need to linearly interpolate desired angles for theta controller

        return traj;
    }
}
