// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.AbstractMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.SwerveTrajectoryWaypoint;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutoCommands {

    public enum StartingTarmac {
        left, right
    };

    public enum StartingPosition {
        left, middle, right
    };

    public enum NumberOfBalls {
        zero, two, three, four, five
    };

    static boolean leftTarmac;
    static String position;
    static int numBalls;

    // "RightMid" - first word: left/right tarmac, second word: left/mid/right
    // positions on tarmac
    // So the above example would be the Right Tarmac in the Middle Position
    private static Map<String, SwerveTrajectoryWaypoint> startPositions = Map.ofEntries(
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>(
            "LeftRight",
            new SwerveTrajectoryWaypoint(6.764, 5.712, 2.035, 2.035) //TODO
        ),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>(
            "LeftMid",
            new SwerveTrajectoryWaypoint(7.022, 4.814, 2.775, 2.775) //TODO
        ),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>(
            "LeftLeft",
            new SwerveTrajectoryWaypoint(5.962, 3.958, 3.141, 3.141) //TODO
        ),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>(
            "RightRight",
            new SwerveTrajectoryWaypoint(8.48, 1.833, -1.561, -2.561) //TODO test
            
        ),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>(
            "RightMid",
            new SwerveTrajectoryWaypoint(7.716, 2.811, -1.920, -1.920) //tested
        ),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>(
            "RightLeft",
            new SwerveTrajectoryWaypoint(6.506, 2.56, -2.283, -1.097) //TODO test
        )
    );
    // shuffleboard value for number of balls, starting Position, left/right tarmac
    // make values to store shuffleboard values in this class
    // make if statements to determine which path is used and return it to
    // robotContainer

    public static SwerveTrajectoryWaypoint getStartingPose(String start) {
        return startPositions.get(start);
    }

    public static void setTarmac(StartingTarmac tarmac) {
        switch (tarmac) {
            case left:
                leftTarmac = true;
                break;
            case right:
                leftTarmac = false;
                break;
            default:
                leftTarmac = false;
                break;
        }
    }

    public static void setPosition(StartingPosition aPosition) {
        switch (aPosition) {
            case left:
                position = "Left";
                break;
            case middle:
                position = "Mid";
                break;
            case right:
                position = "Right";
                break;
            default:
                position = "Left";
                break;
        }
    }

    public static void setBalls(NumberOfBalls balls) {
        switch (balls) {
            case two:
                numBalls = 2;
                break;
            case three:
                numBalls = 3;
                break;
            case four:
                numBalls = 4;
                break;
            case five:
                numBalls = 5;
                break;
            case zero:
                numBalls = 0;
                break;
            default:
                numBalls = 0;
                break;
        }
    }

    public static Command getSelectedAuto(Swerve swerve) {
        
        if (leftTarmac) {
            if (numBalls == 0){
                return new InstantCommand(
                    () -> {
                        var start = getStartingPose("Left" + position);
                        swerve.resetOdometry(start.getPositionAndOrientation());
                    }
                );
            }
            return new LeftTarmacPaths(swerve, position, numBalls);
        } else {
            if (numBalls == 0){
                return new InstantCommand(
                    () -> {
                        var start = getStartingPose("Right" + position);
                        swerve.resetOdometry(start.getPositionAndOrientation());
                    }
                );
            }
            return new RightTarmacPaths(swerve, position, numBalls);
        }
    }
}
