// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//I think this is done

package frc.robot.autos;

import java.util.List;
import java.util.Stack;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.IntakeStates;
import frc.robot.States.ShooterStates;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Bottom5Ball extends SequentialCommandGroup {
  /** Creates a new Bottom5Ball. */
  public Bottom5Ball(Swerve s_Swerve, Shooter m_Shooter) {

  //This goes from the start position to ball 1
  Trajectory bottom5BallPart1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(7.538, 2.988, new Rotation2d(-1.941)),
      List.of(),
      new Pose2d(7.596, 0.849, new Rotation2d(-1.611)),
      Constants.Swerve.trajectoryConfig);
  
  //This goes from ball 1 to ball 2
  Trajectory bottom5BallPart2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(7.596, 0.849, new Rotation2d(-1.611)),
      List.of(new Translation2d(5.029, 1.925)),
      new Pose2d(5.491, 1.626, new Rotation2d(-3.735)),
      Constants.Swerve.trajectoryConfig);

  //This goes from ball 2 to ball 3 (the one in the terminal)
  Trajectory bottom5BallPart3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(5.491, 1.626, new Rotation2d(-3.735)),
      List.of(),
      new Pose2d(1.481, 1.54, new Rotation2d(-2.398)),
      Constants.Swerve.trajectoryConfig);

  //This goes from ball 3 to ball 4
  Trajectory bottom5BallPart4 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.481, 1.54, new Rotation2d(-2.398)),
      List.of(),
      new Pose2d(4.73, 5.677, new Rotation2d(1.149)),
      Constants.Swerve.trajectoryConfig);

  //This goes from ball 4 to a better shooting position
  Trajectory bottom5BallPart5 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(4.73, 5.677, new Rotation2d(1.149)),
      List.of(),
      new Pose2d(4.743, 5.691, new Rotation2d(2.803)),
      Constants.Swerve.trajectoryConfig);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = 
            new SwerveControllerCommand(
                bottom5BallPart1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand2 = 
            new SwerveControllerCommand(
                bottom5BallPart2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand3 = 
            new SwerveControllerCommand(
                bottom5BallPart3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
                
        SwerveControllerCommand swerveControllerCommand4 = 
            new SwerveControllerCommand(
                bottom5BallPart4,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand5 = 
            new SwerveControllerCommand(
                bottom5BallPart5,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
            
      addCommands(
        //Gets the initial pose
        new InstantCommand(() -> s_Swerve.resetOdometry(bottom5BallPart1.getInitialPose())),
        //Deploys the intake
        new InstantCommand(() -> States.deployIntake()),

        //Does the first trajectory while intaking and picks one ball
        new ParallelDeadlineGroup (
          swerveControllerCommand,
          new InstantCommand(() -> States.intake())),

        new InstantCommand(() -> States.stopIntake()),

        //Activates the shooter and shoots the 2 balls
        new InstantCommand(() -> States.activateShooter()),
        new WaitCommand(1.0), 
        
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new InstantCommand(() -> States.feed())),

        new InstantCommand(() -> States.stopIntake()),
        new InstantCommand(() -> States.deactivateShooter()),

        //Does the second and third trajectories while intaking and picks up 2 balls
        new InstantCommand(() -> States.intake()),
        swerveControllerCommand2,
        swerveControllerCommand3,

        new InstantCommand(() -> States.stopIntake()),

        //Activates the shooter and shoots the 2 balls
        new InstantCommand(() -> States.activateShooter()),
        new WaitCommand(1.0), 
        
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new InstantCommand(() -> States.feed())),

        new InstantCommand(() -> States.deactivateShooter()),
        new InstantCommand(() -> States.stopIntake()),

        //Does the fourth trajectory while intaking and picks up 1 ball
        new InstantCommand(() -> States.intake()),
        swerveControllerCommand4,
        new InstantCommand(() -> States.stopIntake()),

        swerveControllerCommand5,

        //Activated the shooter and shoots the ball
        new InstantCommand(() -> States.activateShooter()),
        new WaitCommand(1.0), 
        
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new InstantCommand(() -> States.feed())),

        new InstantCommand(() -> States.deactivateShooter()),
        new InstantCommand(() -> States.stopIntake())
      );
    
  }
  
}

//InstantCommand starts intaking
//path 1
//path 2
//stop intaking
//preshoot
//feed
//stop shooter
//stop intake
