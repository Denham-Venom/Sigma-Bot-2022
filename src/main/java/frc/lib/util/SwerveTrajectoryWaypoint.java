// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class SwerveTrajectoryWaypoint {
    private Pose2d pose;
    private Rotation2d heading;

    public SwerveTrajectoryWaypoint() {
        this.pose = new Pose2d();
        this.heading = new Rotation2d();
    }

    public SwerveTrajectoryWaypoint(Pose2d pose, Rotation2d heading) {
        this.pose = pose;
        this.heading = heading;
    }

    public SwerveTrajectoryWaypoint(Translation2d position, Rotation2d orientation, Rotation2d heading) {
        this.pose = new Pose2d(position, orientation);
        this.heading = heading;
    }

    public SwerveTrajectoryWaypoint(double x, double y, double orientationRadians, double headingRadians) {
        this.pose = new Pose2d(x, y, new Rotation2d(orientationRadians));
        this.heading = new Rotation2d(headingRadians);
    }

    /**
     * Get pose containing position and orientation of point. 
     * @return Pose2d with Translation2d for (x,y) position, Rotation2d for orientation
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Get rotation for direction of movement of waypoint.
     * @return Rotation2d indicating the direction of movement.
     */
    public Rotation2d getHeading() {
        return heading;
    }

    /**
     * Get a pose for the trajectory generator to use as a waypoint.
     * @return Pose2d with (x,y) position in a Translation2d, heading in Rotation2d.
     */
    public Pose2d getTrajectoryPose() {
        return new Pose2d(pose.getTranslation(), heading);
    }
}
