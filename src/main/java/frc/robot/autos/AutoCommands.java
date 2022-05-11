package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.SwerveController.SwerveTrajectory;
import frc.robot.subsystems.Swerve;

public class AutoCommands {

    public static SequentialCommandGroup AutoInit(Swerve s_Swerve, SwerveTrajectory s_trajectory){
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.setGyro(s_trajectory.getInitialRobotHeading())), 
            new InstantCommand(() -> s_Swerve.resetAutoOdometry(s_trajectory.getInitialPose())),
            new InstantCommand(() -> SmartDashboard.putBoolean("resetPath", true))
            // new CheckCalibrated(),
            // new InstantCommand(() -> s_PoseEstimator.sEstimator.resetPosition(s_trajectory.getInitialPose(), s_Swerve.getYaw()))
        );
    }

}
