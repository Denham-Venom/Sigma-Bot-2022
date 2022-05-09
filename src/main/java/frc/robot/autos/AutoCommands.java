// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.AbstractMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.tempDenhamAutoStuff.SwerveTrajectoryWaypoint;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutoCommands {

    private static Swerve swerve;
    private static RightTarmacPaths rightTarmacPaths;
    private static OptimizedRightPathsBlue optimizedRightPathsBlue;


    public enum StartingTarmac {
        left, right
    };

    public enum NumberOfBalls {
        zero, two, three, five
    };

    static boolean leftTarmac;
    static int numBalls;

    // "RightMid" - first word: left/right tarmac, second word: left/mid/right
    // positions on tarmac
    // So the above example would be the Right Tarmac in the Middle Position
    private static Map<String, SwerveTrajectoryWaypoint> startPositions = Map.ofEntries(
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>(
            "Right",
            new SwerveTrajectoryWaypoint(7.524, 1.756, -1.555,  -1.555)
        ),
        new AbstractMap.SimpleImmutableEntry<String, SwerveTrajectoryWaypoint>(
            "Left",
            new SwerveTrajectoryWaypoint(5.308, 5.792, -0.803,  -0.803)
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

    public static void setBalls(NumberOfBalls balls) {
        switch (balls) {
            case two:
                numBalls = 2;
                break;
            case three:
                numBalls = 3;
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

    public static void setSwerve(Swerve swerve) {
        AutoCommands.swerve = swerve;
        rightTarmacPaths = new RightTarmacPaths(swerve);
        optimizedRightPathsBlue = new OptimizedRightPathsBlue(swerve);
    }

    public static Command getSelectedAuto() {
        if(swerve == null) return null;
        if (leftTarmac) {
            var start = getStartingPose("Left");
            if (numBalls == 0){
                return new InstantCommand(
                    () -> {
                        swerve.resetOdometry(start.getPositionAndOrientation());
                    }
                    );
                }
            return new LeftTarmacPaths(swerve, start, numBalls);
        } else {
            var start = getStartingPose("Right");
            if (numBalls == 5){
                var alliance = DriverStation.getAlliance();
                switch(alliance) {
                    case Red:
                        return rightTarmacPaths;
                    case Blue:
                        return optimizedRightPathsBlue;
                    default:
                        return rightTarmacPaths;
                }
            }
            if (numBalls == 0){
                return new InstantCommand(
                    () -> {
                        swerve.resetOdometry(start.getPositionAndOrientation());
                    }
                );
            }
            return new RightTarmacPaths(swerve);
        }
    }
}
