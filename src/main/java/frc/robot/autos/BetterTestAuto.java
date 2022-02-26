// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.HolonomicTrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BetterTestAuto extends SequentialCommandGroup {
  /** Creates a new BetterTestAuto. */
  public BetterTestAuto(Swerve s_Swerve) {

    Pose2d startPos = AutoConstants.startPos;

    List<Pose2d> points = List.of(
      startPos, 
      AutoConstants.rightPoints[0]
    );
    Trajectory traj = HolonomicTrajectoryGenerator.generateTrajectory(points, Constants.Swerve.trajectoryConfig);
    SwerveControllerCommand test = HolonomicTrajectoryGenerator.generateSwerveControllerCommand(
      points,
      traj,
      s_Swerve
      );
    
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
      test
      );
    
  }
}
