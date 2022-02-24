// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.AbstractMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AutoCommands {
    //"RightMid" - first word: left/right tarmac, second word: left/mid/right positions on tarmac
    // So the above example would be the Right Tarmac in the Middle Position
    Map<String, Pose2d> positions = Map.ofEntries(
        // new AbstractMap.SimpleImmutableEntry<String, Pose2d>("LeftLeft",
        // new AbstractMap.SimpleImmutableEntry<String, Pose2d>("LeftMid",
        // new AbstractMap.SimpleImmutableEntry<String, Pose2d>("LeftRight",
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("RightLeft", new Pose2d(6.505, 2.575, new Rotation2d(-2.335))),
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("RightMid", new Pose2d(7.538, 2.988, new Rotation2d(-1.941))),
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("RightRight", new Pose2d(8.476, 1.847, new Rotation2d(-1.561)))
    );

    public void selectPath(String start, int numBalls) {
        
    }

    public Pose2d getStartingPose(String start) {
        return positions.get(start);
    }
}
