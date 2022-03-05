// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.util.SwerveCommandRotation2dSupplierGenerator;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BetterTestAuto extends SequentialCommandGroup {
  /** Creates a new BetterTestAuto. */
  public BetterTestAuto(Swerve s_Swerve) {

    List<Pose2d> points = List.of(
      new Pose2d(3.896, 1.391, new Rotation2d(0)),
      new Pose2d(5.979, 1.462, new Rotation2d(-1.593)),
      new Pose2d(7.788, 1.426, new Rotation2d(0))
    );

    Trajectory traj = TrajectoryGenerator.generateTrajectory(points, Constants.Swerve.trajectoryConfig);
    //Supplier<Rotation2d> rot = SwerveCommandRotation2dSupplierGenerator.generateSwerveCommandRotation2dSupplier(traj, points);
    SwerveControllerCommand move = new SwerveControllerCommand(
      traj,
      s_Swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPXController, 0, 0),
      new PIDController(Constants.AutoConstants.kPYController, 0, 0),
      new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints),
      //rot,
      s_Swerve::setModuleStates,
      s_Swerve);

    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
      move
      );
    
  }
}
