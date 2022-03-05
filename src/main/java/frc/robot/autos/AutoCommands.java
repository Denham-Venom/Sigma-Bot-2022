// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.AbstractMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.SwerveTrajectoryWaypoint;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutoCommands {

    public enum StartingTarmac {left, right};
    public enum StartingPosition {left, middle, right};
    public enum NumberOfBalls {two, three, four, five};

    static boolean leftTarmac;
    static String position;
    static int numBalls;

    //"RightMid" - first word: left/right tarmac, second word: left/mid/right positions on tarmac
    // So the above example would be the Right Tarmac in the Middle Position
    private static Map<String, SwerveTrajectoryWaypoint> startPositions = Map.ofEntries(
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>("LeftLeft", SwerveTrajectoryWaypoint(6.764, 5.712, 2.035, numBalls)),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>("LeftMid", SwerveTrajectoryWaypoint(7.103, 4.871, 2.742, numBalls)),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>("LeftRight", SwerveTrajectoryWaypoint(5.962, 3.958, 3.141, 3.141)),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>("RightLeft", SwerveTrajectoryWaypoint(8.439, 1.876, -1.561 -1.561, numBalls)),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>("RightMid", SwerveTrajectoryWaypoint(7.606, 2.974, -1.894, -1.894)),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>("RightRight", SwerveTrajectoryWaypoint(6.601, 2.546, -2.283, -2.283))
    );
    // shuffleboard value for number of balls, starting Position, left/right tarmac
    // make values to store shuffleboard values in this class
    // make if statements to determine which path is used and return it to robotContainer

    public static SwerveTrajectoryWaypoint getStartingPose(String start) {
        return startPositions.get(start);
    }

    private static SwerveTrajectoryWaypoint SwerveTrajectoryWaypoint(double d, double e, double f, double g) {
        return null;
    }

    public static void setTarmac(StartingTarmac tarmac) {
        switch(tarmac) {
            case left:
            leftTarmac = true;
            case right:
            leftTarmac = false;
        }
    }

    public static void setPosition(StartingPosition aPosition) {
        switch(aPosition) {
            case left:
            position = "Left";
            case middle:
            position = "Mid";
            case right:
            position = "Right";
        }
    }

    public static void setBalls(NumberOfBalls balls) {
        switch(balls) {
            case two:
            numBalls = 2;
            case three:
            numBalls = 3;
            case four:
            numBalls = 4;
            case five:
            numBalls = 5;
        }
    }

    public static Command getSelectedAuto(Swerve swerve) {
        if(leftTarmac) {
            return new LeftTarmacPaths(swerve, position, numBalls);
        } else {
            return new RightTarmacPaths(swerve, position, numBalls);
        }
    }  
}

