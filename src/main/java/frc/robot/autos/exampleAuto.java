package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        //Another list of waypoints
        //wayPoints = listOf(
        //Pose2d(17.285.feet, 19.35.feet, -27.491.degrees),
        //Pose2d(18.367.feet, 16.776.feet, -22.997.degrees),
        //Pose2d(23.143.feet, 14.88.feet, -21.019.degrees)),

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(Math.PI)), config);

        new Rotation2d();
         Trajectory trajectory2 = 
             TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.feetToMeters(23.165), Units.feetToMeters(14.997), Rotation2d.fromDegrees(157.106)),
                List.of( 
                    //new Translation2d( Units.feetToMeters(19.794), Units.feetToMeters(17.525) )
                ),
                new Pose2d(Units.feetToMeters(16.454), Units.feetToMeters(20.215), Rotation2d.fromDegrees(142.0)),
                config);

        new Rotation2d();
            Trajectory trajectory3 = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.feetToMeters(25.484), Units.feetToMeters(9.522), Rotation2d.fromDegrees(-110.0)),
                List.of( 
                    new Translation2d( Units.feetToMeters(24.922), Units.feetToMeters(1.568) )
                ),
                new Pose2d(Units.feetToMeters(16.543), Units.feetToMeters(6.13), Rotation2d.fromDegrees(151.0)),
                config);

        new Rotation2d();
            Trajectory bottomBluePart1 = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.feetToMeters(23.406), Units.feetToMeters(15.653), Rotation2d.fromDegrees(155)),
                List.of( 
                ),
                new Pose2d(Units.feetToMeters(16.187), Units.feetToMeters(20.215), Rotation2d.fromDegrees(154)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                bottomBluePart1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerCommand2 =
                new SwerveControllerCommand(
                    trajectory2,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory2.getInitialPose())),
            swerveControllerCommand
            //swerveControllerCommand2
        );
    }
}
