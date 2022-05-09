// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.tempDenhamAutoStuff;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class SwerveTrajectory {

    private Trajectory trajectory; 
    private Supplier<Rotation2d> angleSupplier;
    
    public SwerveTrajectory(TrajectoryConfig config, SwerveTrajectoryWaypoint... waypoints) {    
        List<Pose2d> pts = new ArrayList<>();
        int numPts = waypoints.length;
        for(int i = 0; i < numPts; i++) {
            pts.add(waypoints[i]);
        }

        trajectory = TrajectoryGenerator.generateTrajectory(pts, config);

        this.angleSupplier = SwerveCommandRotation2dSupplierGenerator.generateSwerveCommandRotation2dSupplier(trajectory, List.of(waypoints));
    }

    public Trajectory getTrajectory() {
        return this.trajectory;
    }

    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }

    public Supplier<Rotation2d> getAngleSupplier() {
        return this.angleSupplier;
    }
}