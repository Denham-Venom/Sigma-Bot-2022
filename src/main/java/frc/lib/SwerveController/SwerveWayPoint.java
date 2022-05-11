package frc.lib.SwerveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class SwerveWayPoint {
    private Pose2d pose;
    private Rotation2d robotHeading;
    
    /**
     * Input a pose and a heading, and this is used with LazySwerveController to generate the trajectory and follow robot heading.
     * @param waypointX
     * @param waypointY
     * @param waypointHeading
     * @param robotHeading
     */
    public SwerveWayPoint(double waypointX, double waypointY, Rotation2d waypointHeading, Rotation2d robotHeading) {
        pose = new Pose2d(waypointX, waypointY, waypointHeading);
        this.robotHeading = robotHeading;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Rotation2d getRobotHeading() {
        return robotHeading;
    }
}
