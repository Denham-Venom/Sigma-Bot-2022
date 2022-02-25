// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//I think this is done

package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Stack;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.Constants.AutoConstants;
import frc.robot.States.IntakeStates;
import frc.robot.States.ShooterStates;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Right5Ball extends SequentialCommandGroup {
  /** Creates a new right5Ball. */
  public Right5Ball(Swerve s_Swerve, Shooter m_Shooter) {

    Pose2d startPos = AutoConstants.startPos;

    //This goes from the start position to ball 1
    Trajectory right5BallPart1 = TrajectoryGenerator.generateTrajectory(
        startPos,
        List.of(),
        AutoConstants.rightPoints [0],
        Constants.Swerve.trajectoryConfig);

    //This turns to ball 2
    Trajectory right5BallPart1T = TrajectoryGenerator.generateTrajectory(
        AutoConstants.rightPoints [1],
        List.of(),
        AutoConstants.rightPoints [2],
        Constants.Swerve.trajectoryConfig);
    
    //This goes from ball 1 to ball 2
    Trajectory right5BallPart2 = TrajectoryGenerator.generateTrajectory(
        AutoConstants.rightPoints [2],
        List.of(),
        AutoConstants.rightPoints [3],
        Constants.Swerve.trajectoryConfig);

    //This turns to ball 3
    Trajectory right5BallPart2T = TrajectoryGenerator.generateTrajectory(
        AutoConstants.rightPoints [3],
        List.of(),
        AutoConstants.rightPoints [4],
        Constants.Swerve.trajectoryConfig);

    //This goes from ball 2 to ball 3 (the one in the terminal)
    Trajectory right5BallPart3 = TrajectoryGenerator.generateTrajectory(
        AutoConstants.rightPoints [4],
        List.of(),
        AutoConstants.rightPoints [5],
        Constants.Swerve.trajectoryConfig);
      
    //right5BallPart2 = right5BallPart2.concatenate(right5BallPart3);
      
    // Trajectory right5BallPart23 = TrajectoryGenerator.generateTrajectory(
    //   List.of(
    //     AutoConstants.rightPoints[0], 
    //     AutoConstants.rightPoints[1], 
    //     AutoConstants.rightPoints[2]
    //   ), Constants.Swerve.trajectoryConfig
    // );

    //This turns to ball 4
    Trajectory right5BallPart3T = TrajectoryGenerator.generateTrajectory(
        AutoConstants.rightPoints [5],
        List.of(),
        AutoConstants.rightPoints [6],
        Constants.Swerve.trajectoryConfig);

    //This goes to ball 4 from ball 3
    Trajectory right5BallPart4 = TrajectoryGenerator.generateTrajectory(
        AutoConstants.rightPoints [6],
        List.of(),
        AutoConstants.rightPoints [7],
        Constants.Swerve.trajectoryConfig);
      
    
    // //This goes from ball 4 to a better shooting position
    // Trajectory right5BallPart5 = TrajectoryGenerator.generateTrajectory(
    //     AutoConstants.rightPoints [3],
    //     List.of(),
    //     AutoConstants.rightPoints [4],
    //     Constants.Swerve.trajectoryConfig);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = 
        new SwerveControllerCommand(
            right5BallPart1,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand1T = 
        new SwerveControllerCommand(
            right5BallPart1T,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand2 = 
        new SwerveControllerCommand(
            right5BallPart2,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand2T = 
        new SwerveControllerCommand(
            right5BallPart2T,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand3 = 
        new SwerveControllerCommand(
            right5BallPart3,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

     SwerveControllerCommand swerveControllerCommand3T = 
        new SwerveControllerCommand(
            right5BallPart3T,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    // SwerveControllerCommand swerveControllerCommand23 = 
    //     new SwerveControllerCommand(
    //         right5BallPart23,
    //         s_Swerve::getPose,
    //         Constants.Swerve.swerveKinematics,
    //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         s_Swerve::setModuleStates,
    //         s_Swerve);
            
    SwerveControllerCommand swerveControllerCommand4 = 
        new SwerveControllerCommand(
            right5BallPart4,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    // SwerveControllerCommand swerveControllerCommand5 = 
    //     new SwerveControllerCommand(
    //         right5BallPart5,
    //         s_Swerve::getPose,
    //         Constants.Swerve.swerveKinematics,
    //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         s_Swerve::setModuleStates,
    //         s_Swerve);
            
    addCommands(
      //Gets the initial pose
      new InstantCommand(() -> s_Swerve.resetOdometry(right5BallPart1.getInitialPose())),
      //Deploys the intake
      new InstantCommand(() -> States.deployIntake()),

      //Picks up ball 1
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

      //Turns to ball 2
      swerveControllerCommand1T,

      //Picks up ball 2 and 3
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand2,
      swerveControllerCommand2T,
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

      //Turns to ball 4
      swerveControllerCommand3T,

      //Picks up ball 4
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand4,
      new InstantCommand(() -> States.stopIntake()),

      //swerveControllerCommand5,

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


// public class enum Pose2d[] {
//    case: blah
//       code;
//    case: blahblah
//       code;
//    return(code);
// }