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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Left5Ball extends SequentialCommandGroup {

  private int waypointIndex;

  /** Creates a new Left5Ball. */
  public Left5Ball(Swerve s_Swerve) {    
    
    Pose2d startPos = AutoConstants.startPos;
    waypointIndex = 0;
    
      Trajectory Left5BallPart1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(7.103, 4.757, new Rotation2d(2.752)),
          List.of(),
          AutoConstants.leftPoints [waypointIndex],
          Constants.Swerve.trajectoryConfig);
          
      Trajectory Left5BallPart2 = TrajectoryGenerator.generateTrajectory(
          List.of(
          AutoConstants.leftPoints [waypointIndex++],
          AutoConstants.leftPoints [waypointIndex++],
          AutoConstants.leftPoints [waypointIndex++],
          AutoConstants.leftPoints [waypointIndex++],
          AutoConstants.leftPoints [waypointIndex]),
          Constants.Swerve.trajectoryConfig);

      Trajectory Left5BallPart3 = TrajectoryGenerator.generateTrajectory(
          AutoConstants.leftPoints [waypointIndex++],
          List.of(),
          AutoConstants.leftPoints [waypointIndex],
          Constants.Swerve.trajectoryConfig);

          var thetaController =
              new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
          SwerveControllerCommand swerveControllerCommand = 
            new SwerveControllerCommand(
                Left5BallPart1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

          SwerveControllerCommand swerveControllerCommand2 = 
            new SwerveControllerCommand(
                Left5BallPart2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

          SwerveControllerCommand swerveControllerCommand3 = 
            new SwerveControllerCommand(
                Left5BallPart3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

      addCommands(
      //Gets the initial pose
      new InstantCommand(() -> s_Swerve.resetOdometry(Left5BallPart1.getInitialPose())),
      //Deploys the intake
      new InstantCommand(() -> States.deployIntake()),

      //Does the first trajectory while intaking and picks one ball
      new InstantCommand(() -> States.intake()),
      
      swerveControllerCommand,

      new InstantCommand(() -> States.stopIntake()),

      //Activates the shooter and shoots the 2 balls
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(1.0), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter()),

      //Does the second trajectory while intaking and picks up 2 balls
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand2,

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
      swerveControllerCommand3,
      new InstantCommand(() -> States.stopIntake()),

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
