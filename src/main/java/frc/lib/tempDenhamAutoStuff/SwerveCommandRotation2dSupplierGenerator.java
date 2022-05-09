// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.tempDenhamAutoStuff;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

/**
 * Static class for generating Rotation2d suppliers for 
 * swerve autonomous commands. This allows for specifying 
 * the rotation of the robot during the path using the orientation 
 * part of the SwerveTrajectoryWaypoint list.
*/
public class SwerveCommandRotation2dSupplierGenerator {

    /**
     * Static method for generating the Rotation2d supplier for a 
     * given trajectory and set of waypoints.
     * @param trajectory The trajectory generated from the trajectory 
     * waypoints.
     * @param trajectoryWaypoints The waypoints used to generate the 
     * trajectory with additional robot orientation information.
     * @return Supplier<Rotation2d> for use in the 
     * SwerveControllerCommand constructor.
     */
    public static Supplier<Rotation2d> generateSwerveCommandRotation2dSupplier(Trajectory trajectory, List<SwerveTrajectoryWaypoint> trajectoryWaypoints) {
        ArrayList<Pose2d> waypoints = new ArrayList<>();
        for(var pt : trajectoryWaypoints) {
            waypoints.add(pt.getPositionAndOrientation());
        }  
        ArrayList<Double> times = new ArrayList<>();
        //first waypoint should occur at time zero so skip
        times.add(0.);

        //loop over states and find points where each waypoints occur TODO - check if waypoints are guaranteed to appear in a trajectory exactly
        List<State> trajStates = trajectory.getStates();
        int numStates = trajStates.size();
        Translation2d curTr = waypoints.get(1).getTranslation();
        for(int i = 0, waypointIndex = 1; i < numStates-1; i++) {
            State state = trajStates.get(i);
            if(state.poseMeters.getTranslation().equals(curTr)) {
                waypointIndex++;
                curTr = waypoints.get(waypointIndex).getTranslation();
                times.add(state.timeSeconds);
            }
        }
        //last waypoint should be last state of trajectory
        times.add(trajStates.get(numStates-1).timeSeconds);

        return new SwerveAngleSupplier(times, waypoints);
    }
}
