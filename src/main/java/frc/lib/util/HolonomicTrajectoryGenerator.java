// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class HolonomicTrajectoryGenerator {
    public static Trajectory generateTrajectory(List<Pose2d> waypoints, TrajectoryConfig config) {
        
        //need at least start and end waypoint
        int numWaypoints = waypoints.size();
        if(numWaypoints < 2) return new Trajectory();

        Pose2d start = waypoints.get(0);
        Pose2d afterStart = waypoints.get(1);
        Pose2d end = waypoints.get(numWaypoints-1);
        Pose2d beforeEnd = waypoints.get(numWaypoints-2);

        //calculate rotations to be used for generating spline
        //start is angle between 1st and 2nd waypoint
        Rotation2d startRot = new Rotation2d(Math.atan((afterStart.getY() - start.getY()) / (afterStart.getX() - start.getX())));
        //end is angle between penultimate and last waypoint
        Rotation2d endRot = new Rotation2d(Math.atan((end.getY() - beforeEnd.getY()) / (end.getX() - beforeEnd.getX())));
        Rotation2d rot = new Rotation2d(Math.atan( (end.getY() - start.getY()) / (end.getX() - start.getX()) ));
        //make new poses to use to spline generation with modified angles
        Pose2d newStart = new Pose2d(start.getTranslation(), rot);
        Pose2d newEnd = new Pose2d(end.getTranslation(), rot);

        //convert interior waypoint Pose2d's into translations for spline generation
        List<Translation2d> interiorWaypoints = new ArrayList<>();
        for(int i = 1; i < numWaypoints-1; i++) {
            var interior = waypoints.get(i);
            interiorWaypoints.add(new Translation2d(interior.getX(), interior.getY()));
        }

        Trajectory traj = TrajectoryGenerator.generateTrajectory(newStart, interiorWaypoints, newEnd, config);
        return traj;
    }

    public static SwerveControllerCommand generateSwerveControllerCommand(List<Pose2d> waypoints, Trajectory traj, Swerve s_Swerve) {
        //Trajectory traj = HolonomicTrajectoryGenerator.generateTrajectory(waypoints, config);
        int numWaypoints = waypoints.size();
        List<State> trajStates = traj.getStates();
        int numStates = trajStates.size();
        List<State> newStates = new ArrayList<State>();
        int stateIndex = 0;
        List<Double> times = new ArrayList<>();
        for(int i = 0; i < numWaypoints-1; i++) {
            Translation2d pos = waypoints.get(i).getTranslation();
            boolean found = false;
            while(!found) {
                if(stateIndex < numStates) {
                    State curState = trajStates.get(stateIndex);
                    stateIndex++;
                    if(curState.poseMeters.getTranslation().equals(pos)) {
                        found = true;
                        times.add(curState.timeSeconds);
                    }
                } else {
                    //if reach end of states, will act like going from init orientation to final for lerp purposes
                    times.add((double) 0);
                }
            }
        }
        int timeIndex = 1;
        int numTimes = times.size();
        Rotation2d lerpStart = waypoints.get(0).getRotation();
        Rotation2d lerpEnd = waypoints.get(1).getRotation();
        for(var state : trajStates) {
            double time = state.timeSeconds;
            if(time >= times.get(timeIndex)) {
                timeIndex++;
                if(timeIndex == numTimes) {
                    lerpStart = waypoints.get(0).getRotation();
                    lerpEnd = waypoints.get(numWaypoints-1).getRotation();
                } 
                lerpStart = waypoints.get(timeIndex-1).getRotation();
                lerpEnd = waypoints.get(timeIndex).getRotation();
            }
            double stateRot = lerpStart.getRadians() + (lerpEnd.getRadians() - lerpStart.getRadians()) * time;
            State newState = new State(
                time,
                state.velocityMetersPerSecond, //doesn't really matter
                state.accelerationMetersPerSecondSq, //doesn't really matter
                new Pose2d(
                    state.poseMeters.getX(), 
                    state.poseMeters.getY(), 
                    new Rotation2d(stateRot)),
                state.curvatureRadPerMeter); //doesn't really matter
            newStates.add(newState);
        }
        Trajectory turnTraj = new Trajectory(newStates);
        Supplier<Rotation2d> ang = getSupplier(turnTraj);

        return new SwerveControllerCommand(
            traj,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            Constants.Swerve.thetaController,
            ang,
            s_Swerve::setModuleStates,
            s_Swerve);
    }

    private static Supplier<Rotation2d> getSupplier(Trajectory traj) {
        Timer timer = new Timer();
        boolean timerStarted = false;
        //Supplier<Rotation2d> -- returns the next specified heading of the robot in the path
        Supplier<Rotation2d> ang = () -> {
            if(!timerStarted) {
                timer.reset();
                timer.start();
            }
            double curTime = timer.get();
            return traj.sample(curTime).poseMeters.getRotation();
        };

        return ang;
    }
}
