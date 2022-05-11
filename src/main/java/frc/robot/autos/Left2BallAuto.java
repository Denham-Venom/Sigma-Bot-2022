package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.SwerveController.SwerveController;
import frc.lib.SwerveController.SwerveControllerConfig;
import frc.lib.SwerveController.SwerveTrajectory;
import frc.lib.SwerveController.SwerveWayPoint;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.subsystems.Swerve;

public class Left2BallAuto extends SequentialCommandGroup {

    public Left2BallAuto(Swerve s_Swerve){

        SwerveControllerConfig controllerConfig = new SwerveControllerConfig(
            s_Swerve,
            s_Swerve::getAutoPose,
            s_Swerve::setModuleStates,
            Constants.Swerve.swerveKinematics,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared,
            Constants.AutoConstants.kPXController,
            Constants.AutoConstants.kPYController,
            Constants.AutoConstants.kPThetaController
        );

        SwerveTrajectory tarmacToWall = new SwerveTrajectory(
            List.of(
                new SwerveWayPoint(5.308, 5.792, Rotation2d.fromDegrees(-89.094), Rotation2d.fromDegrees(-46.00)), 
                new SwerveWayPoint(6.023, 5.095, Rotation2d.fromDegrees(-45.50), Rotation2d.fromDegrees(-45.50))
            ), 
            new TrajectoryConfig(3.0, 3.0).setKinematics(Constants.Swerve.swerveKinematics), controllerConfig
        );

        addCommands(
            /* Set odometry and poseestimator to the startiong position, and check hood calibrated */
            AutoCommands.AutoInit(s_Swerve, tarmacToWall),
            
            /* Deploy Intake */
            new InstantCommand(() -> States.deployIntake()),

            /* Deploy Intake and Pickup 1 ball */
            new InstantCommand(() -> States.intake()),

            new SwerveController(tarmacToWall),

            new InstantCommand(() -> States.stopIntake()),

            /* Activate the shooter and shoot the 2 balls */
            new InstantCommand(() -> States.activateShooter()),
            new WaitCommand(1.0), 

            new ParallelDeadlineGroup(
            new WaitCommand(1),
            new InstantCommand(() -> States.feed())),

            /* Stop Shooting */
            new InstantCommand(() -> States.stopIntake()),
            new InstantCommand(() -> States.deactivateShooter())
        );
    }
}
