// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.tempDenhamAutoStuff;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** 
 * Class for representing waypoints used in trajectory 
 * waypoints while also storing additional information 
 * regarding point orientation angle used to control 
 * robot angle during pathing. This class returns 
 * values by reference without copying.
*/
public class SwerveTrajectoryWaypoint extends Pose2d {

    // Contains angle for direction point is facing
    private Rotation2d orientation;

    /**
     * Create an empty SwerveTrajectoryWaypoint with 
     * default values for position, heading, and 
     * orientation (all 0).
     */
    public SwerveTrajectoryWaypoint() {
        super();
        this.orientation = new Rotation2d();
    }

    /**
     * Creates a SwerveTrajectoryWaypoint using the 
     * provided position, orientation, and heading.
     * @param position Translation2d representing (x,y) 
     * position of the point.
     * @param orientation Rotation2d representing the angle 
     * the point is facing.
     * @param heading Rotation2d representing the direction of 
     * motion of the point.
     */
    public SwerveTrajectoryWaypoint(Translation2d position, Rotation2d orientation, Rotation2d heading) {
        super(position, heading);
        this.orientation = orientation;
    }

    /**
     * Creates a SwerveTrajectoryWaypoint using the 
     * provided position, orientation, and heading.
     * @param x The position along the x axis.
     * @param y The position along the y axis.
     * @param orientationRadians The angle the point is 
     * facing in radians.
     * @param headingRadians The angle of the direction of 
     * motion of the point in radians.
     */
    public SwerveTrajectoryWaypoint(double x, double y, double orientationRadians, double headingRadians) {
        super(x, y, new Rotation2d(headingRadians));
        this.orientation = new Rotation2d(orientationRadians);
    }

    /**
     * Gets the orientation angle of this waypoint.
     * @return Rotation2d representing angle.
     */
    public Rotation2d getOrientation() {
        return this.orientation;
    }

    /**
     * Gets a pose representing the (x,y) position of 
     * this waypoint and the orientation angle of the 
     * waypoint.
     * @return Pose2d from position and orientation.
     */
    public Pose2d getPositionAndOrientation() {
        return new Pose2d(this.getTranslation(), orientation);
    }
}
