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
    static Map<String, Pose2d> positions = Map.ofEntries(
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("LeftLeft", new Pose2d(6.764, 5.712, new Rotation2d(2.035))),
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("LeftMid", new Pose2d(7.103, 4.871, new Rotation2d(2.742))),
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("LeftRight", new Pose2d(5.962, 3.958, new Rotation2d(3.141))),
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("RightLeft", new Pose2d(8.439, 1.876, new Rotation2d(-1.561))),
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("RightMid", new Pose2d(7.606, 2.974, new Rotation2d(-1.894))),
        new AbstractMap.SimpleImmutableEntry<String, Pose2d>("RightRight", new Pose2d(6.601, 2.546, new Rotation2d(-2.283)))
    );

    public static Pose2d getStartingPose(String start) {
        return positions.get(start);
    }
}
