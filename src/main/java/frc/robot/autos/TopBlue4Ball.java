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
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TopBlue4Ball extends SequentialCommandGroup {
  /** Creates a new BottomBlue4Ball. */
  public TopBlue4Ball(Swerve s_Swerve) {
      Trajectory topBlue4Ball = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(7.152, 4.778, new Rotation2d(2.73)),
          new Pose2d(4.921, 6.133, new Rotation2d(2.617)),
          new Pose2d(5.094, 1.84, new Rotation2d(0.754)),
          new Pose2d(7.584, 0.456, new Rotation2d(-0.009))
          ),

        Constants.Swerve.trajectoryConfig);
  
          var thetaController =
              new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
          SwerveControllerCommand swerveControllerCommand = 
            new SwerveControllerCommand(
                topBlue4Ball,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
      addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(topBlue4Ball.getInitialPose())));
        //new ParallelDeadlineGroup(
          //swerveControllerCommand,
          //new SequentialCommandGroup(
            //new InstantCommand(() -> m_Intake.deployIntake()),
            //new StartEndCommand(
              //() -> States.intakeState = IntakeStates.intaking,
              //() -> States.intakeState = IntakeStates.disabled)
          //)
        //),
        //new findTargetAndShoot()
      //);
      
  
  }
}
