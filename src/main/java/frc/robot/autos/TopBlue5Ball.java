// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TopBlue5Ball extends SequentialCommandGroup {
  /** Creates a new BottomBlue5Ball. */
  public TopBlue5Ball(Swerve s_Swerve) {
    Trajectory topBlue5Ball = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(7.158, 4.843, new Rotation2d(2.704)),
        new Pose2d(5.041, 6.126, new Rotation2d(-0.37)),
        new Pose2d(1.346, 1.383, new Rotation2d(-2.365)),
        new Pose2d(5.029, 1.868, new Rotation2d(0)),
        new Pose2d(7.584, 0.485, new Rotation2d(0))
        ),
        Constants.Swerve.trajectoryConfig);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = 
            new SwerveControllerCommand(
                topBlue5Ball,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
    addCommands(new InstantCommand( () -> s_Swerve.resetOdometry(topBlue5Ball.getInitialPose()) ),
      swerveControllerCommand
      );
    
  }
  
}
