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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Bottom5BallRight extends SequentialCommandGroup {
  /** Creates a new Bottom5BallRight. */
  public Bottom5BallRight(Swerve s_Swerve, Shooter m_Shooter) {
    
  Trajectory bottom5BallRightPart1 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(27.809, 6.014, new Rotation2d(-90.717)),
    List.of(),
    new Pose2d(24.833, 1.428, new Rotation2d(-109.65)),
    Constants.Swerve.trajectoryConfig);

  Trajectory bottom5BallRightPart2 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(24.833, 1.428, new Rotation2d(-109.65)),
    List.of(),
    new Pose2d(16.633, 6.224, new Rotation2d(-118.923)),
    Constants.Swerve.trajectoryConfig);

  Trajectory bottom5BallRightPart3 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(16.633, 6.224, new Rotation2d(-118.923)),
    List.of(),
    new Pose2d(3.969, 4.118, new Rotation2d(-138.52)),
    Constants.Swerve.trajectoryConfig);

  Trajectory bottom5BallRightPart4 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(3.969, 4.118, new Rotation2d(-138.52)),
    List.of(),
    new Pose2d(16.097, 20.028, new Rotation2d(62.148)),
    Constants.Swerve.trajectoryConfig);

      var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = 
        new SwerveControllerCommand(
            bottom5BallRightPart1,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

        SwerveControllerCommand swerveControllerCommand2 = 
        new SwerveControllerCommand(
            bottom5BallRightPart2,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

        SwerveControllerCommand swerveControllerCommand3 = 
        new SwerveControllerCommand(
            bottom5BallRightPart3,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

        SwerveControllerCommand swerveControllerCommand4 = 
        new SwerveControllerCommand(
            bottom5BallRightPart4,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

  

    addCommands();
  }
}
