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
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Right2Ball extends SequentialCommandGroup {

  private int waypointIndex;

  /** Creates a new Right2Ball. */
  public Right2Ball(Swerve s_Swerve, Shooter m_Shooter, Intaker m_Intaker) {

    Pose2d startPos = AutoCommands.getStartingPose("RightMid");
    waypointIndex = 0;

      Trajectory right2Ball = TrajectoryGenerator.generateTrajectory(
        startPos,
        List.of(),
        AutoConstants.rightPoints [waypointIndex],
        Constants.Swerve.trajectoryConfig);
  
          var thetaController =
              new ProfiledPIDController(
                  Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
          SwerveControllerCommand swerveControllerCommand = 
              new SwerveControllerCommand(
                  right2Ball,
                  s_Swerve::getPose,
                  Constants.Swerve.swerveKinematics,
                  new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                  new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                  thetaController,
                  s_Swerve::setModuleStates,
                  s_Swerve);

      addCommands(
        //Gets the initial pose
        new InstantCommand(() -> s_Swerve.resetOdometry(right2Ball.getInitialPose())),
        //Deploys the intake 
        new InstantCommand(() -> States.deployIntake()),
        
        //Picks up ball 1 while intaking 
        new ParallelDeadlineGroup(
          swerveControllerCommand, 
          new InstantCommand(() -> States.intake())),

        new InstantCommand(() -> States.stopIntake()),

        //Activates the shooter and shoots 2 balls        
        new InstantCommand(() -> States.activateShooter()),
        new WaitCommand(1.0), 
        
        new ParallelDeadlineGroup(
          new WaitCommand(2.5),
          new InstantCommand(() -> States.feed())),

        //Stops the shooter and intake
        new InstantCommand(() -> States.deactivateShooter()),
        new InstantCommand(() -> States.intake()));
  
  }
}
