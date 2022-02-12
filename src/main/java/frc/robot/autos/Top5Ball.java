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
public class Top5Ball extends SequentialCommandGroup {
  /** Creates a new Top5Ball. */
  public Top5Ball(Swerve s_Swerve) {
      Trajectory top5Ball = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(7.152, 4.778, new Rotation2d(2.73)),
          new Pose2d(4.921, 6.133, new Rotation2d(2.617)),
          new Pose2d(1.37, 1.398, new Rotation2d(0.754)),
          new Pose2d(5.042, 1.897, new Rotation2d(0.262)),
          new Pose2d(7.598, 0.499, new Rotation2d(0))
          ),
          Constants.Swerve.trajectoryConfig);
  
          var thetaController =
              new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
          SwerveControllerCommand swerveControllerCommand = 
            new SwerveControllerCommand(
                top5Ball,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
      addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(top5Ball.getInitialPose())),
        swerveControllerCommand);
      
  
  }
}
