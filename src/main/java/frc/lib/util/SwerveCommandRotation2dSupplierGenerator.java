// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

/** Add your docs here. */
public class SwerveCommandRotation2dSupplierGenerator {

    public static Supplier<Rotation2d> generateSwerveCommandRotation2dSupplier(Trajectory trajectory, List<Pose2d> waypoints) {

        return () -> new Rotation2d();
    }
}
