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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Bottom5BallRight extends SequentialCommandGroup {
  /** Creates a new Bottom5BallRight. */
  public Bottom5BallRight(Swerve s_Swerve, Shooter m_Shooter) {
    
  Trajectory bottom5BallRightPart1 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(8.476, 1.847, new Rotation2d(-1.561)),
    List.of(),
    new Pose2d(7.528, 0.393, new Rotation2d(1.518)),
    Constants.Swerve.trajectoryConfig);

  Trajectory bottom5BallRightPart2 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(7.528, 0.393, new Rotation2d(1.518)),
    List.of(),
    new Pose2d(5.029, 1.925, new Rotation2d(2.234)),
    Constants.Swerve.trajectoryConfig);

  Trajectory bottom5BallRightPart3 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(5.029, 1.925, new Rotation2d(2.234)),
    List.of(),
    new Pose2d(1.305, 1.398, new Rotation2d(-2.391)),
    Constants.Swerve.trajectoryConfig);

  Trajectory bottom5BallRightPart4 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(1.305, 1.398, new Rotation2d(-2.391)),
    List.of(),
    new Pose2d(4.866, 6.033, new Rotation2d(1.659)),
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

  

    addCommands(
        //Gets the initial pose
        new InstantCommand(() -> s_Swerve.resetOdometry(bottom5BallRightPart1.getInitialPose())),
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
          new WaitCommand(2.5),
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
          new WaitCommand(2.5),
          new InstantCommand(() -> States.feed())),

        new InstantCommand(() -> States.deactivateShooter()),
        new InstantCommand(() -> States.stopIntake()),

        //Does the fourth trajectory while intaking and picks up 1 ball
        new ParallelDeadlineGroup(
          swerveControllerCommand4,
          new InstantCommand(() -> States.intake())),

        new InstantCommand(() -> States.stopIntake()),

        //Activated the shooter and shoots the ball
        new InstantCommand(() -> States.activateShooter()),
        new WaitCommand(1.0), 
        
        new ParallelDeadlineGroup(
          new WaitCommand(2.5),
          new InstantCommand(() -> States.feed())),

        new InstantCommand(() -> States.deactivateShooter()),
        new InstantCommand(() -> States.stopIntake())
      );
  }
}
