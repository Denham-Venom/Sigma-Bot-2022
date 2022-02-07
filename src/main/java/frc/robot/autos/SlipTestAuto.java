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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlipTestAuto extends SequentialCommandGroup {
  /** Creates a new SlipTestAuto. */
  public SlipTestAuto(Swerve s_Swerve) {
    Trajectory slipTestAuto = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(5.113, 3.466, new Rotation2d(3.133)),
        new Pose2d(3.589, 3.466, new Rotation2d(3.142)),
        new Pose2d(4.36, 4.179, new Rotation2d(-3.119))
        ),
        Constants.Swerve.trajectoryConfig);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = 
            new SwerveControllerCommand(
                slipTestAuto,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
    addCommands(
      new InstantCommand( () -> s_Swerve.resetOdometry(slipTestAuto.getInitialPose()) ),
      swerveControllerCommand
    );
    
  }
  
}
