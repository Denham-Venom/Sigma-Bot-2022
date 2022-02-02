// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.Constants.Shooter;
import frc.robot.States.ShooterStates;
import frc.robot.commands.Intake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetBallTopBlue extends SequentialCommandGroup {
  /** Creates a new GetBallTopBlue. */
  public GetBallTopBlue(Swerve s_Swerve) {
    // This will load the file "getBallSwerve.path" and generate it with a max velocity of 3 m/s and a max acceleration of 2 m/s^2
    PathPlannerTrajectory getAndShootBall = PathPlanner.loadPath("getBallSwerve", 3, 2);

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation
    // from the PathPlannerTrajectory to control the robot's rotation.
    // See the WPILib SwerveControllerCommand for more info on what you need to pass to the command
    // PPSwerveControllerCommand command = new PPSwerveControllerCommand(
    //   getAndShootBall,
    //   poseSupplier,
    //   kinematics,
    //   xController,
    //   yController,
    //   thetaController,
    //   outputModuleStates,
    //   requirements
    // );

    var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
        getAndShootBall,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);

    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(getAndShootBall.getInitialPose())),
            swerveControllerCommand

            // /* Shoot from Starting Point */
            // new InstantCommand(() -> States.shooterState = ShooterStates.preShoot),
            // new WaitCommand(1.0), //Wait for 1 second to auto aim and spin up before shooting
            // new ParallelDeadlineGroup(
            //     new WaitCommand(2.5),
            //     new Shoot(1.0),
            //     new Intake(m_Intaker)
            // ),
            // new InstantCommand(() -> States.shooterState = ShooterStates.disabled)
    );
  }
}
