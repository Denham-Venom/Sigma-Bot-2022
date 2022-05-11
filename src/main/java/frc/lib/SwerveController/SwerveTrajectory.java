package frc.lib.SwerveController;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class SwerveTrajectory {
    private List<SwerveWayPoint> wayPoints;
    private TrajectoryConfig trajConfig;
    private SwerveControllerConfig controllerConfig;    
    
    public SwerveTrajectory(List<SwerveWayPoint> wayPoints, TrajectoryConfig trajConfig, SwerveControllerConfig controllerConfig) {
        this.wayPoints = wayPoints;
        this.trajConfig = trajConfig;
        this.controllerConfig = controllerConfig;
    }

    public List<SwerveWayPoint> getWayPoints(){
        return wayPoints;
    }

    public TrajectoryConfig getTrajectoryConfig(){
        return trajConfig;
    }

    public SwerveControllerConfig getControllerConfig(){
        return controllerConfig;
    }

    public double getInitialRobotHeading(){
        return wayPoints.get(0).getRobotHeading().getDegrees();
    }

    public Pose2d getInitialPose(){
        return wayPoints.get(0).getPose();
    }

    public SwerveWayPoint getLastWayPoint(){
        return wayPoints.get(wayPoints.size() - 1);
    }
}
