// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SwerveCommandRotation2dSupplierGenerator {

    public static Supplier<Rotation2d> generateSwerveCommandRotation2dSupplier(Trajectory trajectory, List<Pose2d> waypoints) {
        ArrayList<Double> times = new ArrayList<>();
        List<State> trajStates = trajectory.getStates();
        int numStates = trajStates.size();
        for(int i = 0, waypointIndex = 0; i < numStates; i++) {
            
        }

        return () -> new Rotation2d();
    }

    // private static Supplier<Rotation2d> getSupplier() {
    //     Timer timer = new Timer();
    //     boolean timerStarted = false;
    //     //Supplier<Rotation2d> -- returns the next specified heading of the robot in the path
    //     Supplier<Rotation2d> ang = () -> {
    //         if(!timerStarted) {
    //             timer.reset();
    //             timer.start();
    //         }
    //         double curTime = timer.get();
    //         return traj.sample(curTime).poseMeters.getRotation();
    //     };

    //     return ang;
    // }
}
