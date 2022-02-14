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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Top4Ball extends SequentialCommandGroup {
  /** Creates a new Top4Ball. */
  public Top4Ball(Swerve s_Swerve, Shooter m_Shooter) {

      Trajectory top4BallPart1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(4.866, 6.033, new Rotation2d(1.659)),
          List.of(),
          new Pose2d(4.921, 6.133, new Rotation2d(2.617)),
          Constants.Swerve.trajectoryConfig);

      Trajectory top4BallPart2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(4.921, 6.133, new Rotation2d(2.617)),
        List.of(),
        new Pose2d(5.094, 1.84, new Rotation2d(0.754)),
        Constants.Swerve.trajectoryConfig);

      Trajectory top4BallPart3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.094, 1.84, new Rotation2d(0.754)),
        List.of(),
        new Pose2d(7.584, 0.456, new Rotation2d(-0.009)),
        Constants.Swerve.trajectoryConfig);

  
          var thetaController =
              new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
          SwerveControllerCommand swerveControllerCommand = 
            new SwerveControllerCommand(
                top4BallPart1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand2 = 
            new SwerveControllerCommand(
                top4BallPart2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

      SwerveControllerCommand swerveControllerCommand3 = 
            new SwerveControllerCommand(
                top4BallPart3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

      addCommands(
        //Gets the initial pose 
        new InstantCommand(() -> s_Swerve.resetOdometry(top4BallPart1.getInitialPose())),
        //Deploys the intake
        new InstantCommand(() -> States.deployIntake()),

        //Does the first trajectory while intaking and picks up 1 ball
        new InstantCommand(() -> States.intake()),
        swerveControllerCommand,

        new InstantCommand(() -> States.stopIntake()),

        //Activates the shooter and shoots 2 balls 
        new InstantCommand(() -> States.activateShooter()),
        new WaitCommand(1.0),

        new ParallelDeadlineGroup(
            new WaitCommand(2.5),
            new InstantCommand(() -> States.feed())),
        
        new InstantCommand(() -> States.deactivateShooter()),
        new InstantCommand(() -> States.stopIntake()),

        //Does the second and third trajectory while intaking and picks up 2 balls
        new InstantCommand(() -> States.intake()),
        swerveControllerCommand2,
        swerveControllerCommand3,
  
        new InstantCommand(() -> States.stopIntake()),

        //Activates the shooter and shoots 2 balls
        new InstantCommand(() -> States.activateShooter()),
        new WaitCommand(1.0),

        new ParallelDeadlineGroup(
            new WaitCommand(2.5),
            new InstantCommand(() -> States.feed())),
        
        new InstantCommand(() -> States.stopIntake()),
        new InstantCommand(() -> States.deactivateShooter())
      );
        
      
  
  }
}
