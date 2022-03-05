// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Stack;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.Timer;
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

  //private Timer m_timer;
  private int waypointIndex;

  /** Creates a new right5Ball. */
  public Right5Ball(Swerve s_Swerve, String position) {

    //m_timer = new Timer();
    waypointIndex = 0;
    Pose2d startPos = AutoCommands.getStartingPose("Right" + position);

    //This goes from the start position to ball 1
    Trajectory right5BallPart1 = TrajectoryGenerator.generateTrajectory(
        startPos,
        List.of(),
        AutoConstants.rightPoints [waypointIndex],
        Constants.Swerve.trajectoryConfig);
    
    //This goes from ball 1 to ball 2
    Trajectory right5BallPart2 = TrajectoryGenerator.generateTrajectory(
        List.of(
        AutoConstants.rightPoints [waypointIndex++],
        AutoConstants.rightPoints [waypointIndex++],
        AutoConstants.rightPoints [waypointIndex++],
        AutoConstants.rightPoints [waypointIndex]),
        Constants.Swerve.trajectoryConfig);

    //This goes to ball 4 from ball 3
    Trajectory right5BallPart3 = TrajectoryGenerator.generateTrajectory(
        List.of(
        AutoConstants.rightPoints [waypointIndex++],
        AutoConstants.rightPoints [waypointIndex++],
        AutoConstants.rightPoints [waypointIndex]),
        Constants.Swerve.trajectoryConfig);
      
    
    //This goes from ball 4 to a better shooting position
    Trajectory right5BallPart4 = TrajectoryGenerator.generateTrajectory(
        AutoConstants.rightPoints [waypointIndex++],
        List.of(),
        AutoConstants.rightPoints [waypointIndex],
        Constants.Swerve.trajectoryConfig);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // List<Double> waypointTimeStamps = new ArrayList<Double>();
    // waypointIndex = 0;
    // Supplier<Rotation2d> rotSupplier = () -> {
    //   double curTime = m_timer.get();
    //   if(curTime >= waypointTimeStamps.get(waypointIndex)) {
    //     waypointIndex++;
    //   }
    //   return Constants.AutoConstants.rightPoints[waypointIndex].getRotation();
    // };

    SwerveControllerCommand swerveControllerCommand1 = 
        new SwerveControllerCommand(
            right5BallPart1,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand2 = 
        new SwerveControllerCommand(
            right5BallPart2,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand3 = 
        new SwerveControllerCommand(
            right5BallPart3,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints),
            s_Swerve::setModuleStates,
            s_Swerve);
            
    SwerveControllerCommand swerveControllerCommand4 = 
        new SwerveControllerCommand(
            right5BallPart4,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints),
            s_Swerve::setModuleStates,
            s_Swerve);
            
    addCommands(
      //Gets the initial pose
      new InstantCommand(() -> s_Swerve.resetOdometry(right5BallPart1.getInitialPose())),
      //Deploys the intake
      new InstantCommand(() -> States.deployIntake()),

      //Picks up ball 1  
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand1,
      new InstantCommand(() -> States.stopIntake()),

      //Activates the shooter and shoots the 2 balls
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(1.0), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter()),

      //Picks up ball 2 and 3
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

      //Picks up ball 4
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand3,
      new InstantCommand(() -> States.stopIntake()),

      //Turns to the goal
      swerveControllerCommand4,

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