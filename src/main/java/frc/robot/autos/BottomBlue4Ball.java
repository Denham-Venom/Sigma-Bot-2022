// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import com.fasterxml.jackson.databind.node.IntNode;

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
import frc.robot.States.IntakeExtendStates;
import frc.robot.States.ShooterStates;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomBlue4Ball extends SequentialCommandGroup {
  /** Creates a new BottomBlue4Ball. */
  public BottomBlue4Ball(Swerve s_Swerve) {
      Trajectory bottomBlue4BallPart1 = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(4.97, 6.254, new Rotation2d(-1.948)),
            new Pose2d(5, 1.819, new Rotation2d(1.655))
            ),
            Constants.Swerve.trajectoryConfig);
      Trajectory bottomBlue4BallPart2 = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(5, 1.819, new Rotation2d(1.655)),
            new Pose2d(7.584, 0.456, new Rotation2d(-1.62))
            ),
            Constants.Swerve.trajectoryConfig);
      Trajectory bottomBlue4BallPart3 = TrajectoryGenerator.generateTrajectory(
        List.of(
            new Pose2d(7.584, 0.456, new Rotation2d(-1.62)),
            new Pose2d(7.502, 3.024, new Rotation2d(1.167))
            ),
            Constants.Swerve.trajectoryConfig);
  
          var thetaController =
              new ProfiledPIDController(
                  Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
          SwerveControllerCommand swerveControllerCommand = 
              new SwerveControllerCommand(
                  bottomBlue4BallPart1,
                  s_Swerve::getPose,
                  Constants.Swerve.swerveKinematics,
                  new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                  new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                  thetaController,
                  s_Swerve::setModuleStates,
                  s_Swerve);
          SwerveControllerCommand swerveControllerCommand2 = 
              new SwerveControllerCommand(
                  bottomBlue4BallPart2,
                  s_Swerve::getPose,
                  Constants.Swerve.swerveKinematics,
                  new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                  new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                  thetaController,
                  s_Swerve::setModuleStates,
                  s_Swerve);
          SwerveControllerCommand swerveControllerCommand3 = 
              new SwerveControllerCommand(
                  bottomBlue4BallPart3,
                  s_Swerve::getPose,
                  Constants.Swerve.swerveKinematics,
                  new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                  new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                  thetaController,
                  s_Swerve::setModuleStates,
                  s_Swerve);
      addCommands(
        //new InstantCommand(() -> s_Swerve.resetOdometry(bottomBlue4BallPart1.getInitialPose())),
        //new InstantCommand(() -> States.IntakeExtendStates = IntakeExtendStates.deployIntake),

        //new ParallelDeadlineGroup(
          //swerveControllerCommand,
          //new Command(() -> States.IntakeStates = IntakeStates.intaking),

        //new InstantCommand(() -> States.shooterState = ShooterStates.preShoot),
        //new WaitCommand(1.0), 
        
        //new ParallelDeadlineGroup(
            //new WaitCommand(2.5),
            //new Shoot(m_Shooter, 1.0),
            //new Intake(m_Intaker)),
        
        //new ParallelDeadlineGroup(
          //swerveControllerCommand2,
          //new Intake(m_Intaker)),
        
        //new ParallelDeadlineGroup(
          //swerveControllerCommand3,
          //new Intake(m_Intaker)),

        //new ParallelDeadlineGroup(
          //swerveControllerCommand,
          //new Intake(m_Intaker)),
  
        //new InstantCommand(() -> States.shooterState = ShooterStates.preShoot),
        //new WaitCommand(1.0)
      );

        
      
  
  }
}
