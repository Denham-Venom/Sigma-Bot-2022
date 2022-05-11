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

public class FiveBallAuto extends SequentialCommandGroup {

    public FiveBallAuto(Swerve s_Swerve){

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
                new SwerveWayPoint(7.524, 1.756, Rotation2d.fromDegrees(-89.094), Rotation2d.fromDegrees(-89.094)), 
                new SwerveWayPoint(7.608, 0.929, Rotation2d.fromDegrees(-90.011), Rotation2d.fromDegrees(-90.011))
            ), 
            new TrajectoryConfig(3.0, 3.0).setKinematics(Constants.Swerve.swerveKinematics), controllerConfig
        );
        
        SwerveTrajectory wallToCenter = new SwerveTrajectory(
            List.of( 
                new SwerveWayPoint(7.608, 0.929, Rotation2d.fromDegrees(148.79), Rotation2d.fromDegrees(-90.011)),
                new SwerveWayPoint(5.355, 2.026, Rotation2d.fromDegrees(136.3), Rotation2d.fromDegrees(-170.454))
            ), 
            new TrajectoryConfig(3.0, 3.0).setKinematics(Constants.Swerve.swerveKinematics), controllerConfig
        );

        SwerveTrajectory centerToHpStation = new SwerveTrajectory(
            List.of( 
                new SwerveWayPoint(5.355, 2.026, Rotation2d.fromDegrees(-170.45), Rotation2d.fromDegrees(-170.45)),
                new SwerveWayPoint(1.467, 1.512, Rotation2d.fromDegrees(-172.17), Rotation2d.fromDegrees(-157.22))
            ), 
            new TrajectoryConfig(3.0, 3.0).setKinematics(Constants.Swerve.swerveKinematics), controllerConfig
        );

        SwerveTrajectory hpStationToCenter = new SwerveTrajectory(
            List.of(
                new SwerveWayPoint(1.467, 1.512, Rotation2d.fromDegrees(41.48), Rotation2d.fromDegrees(-157.22)),
                new SwerveWayPoint(3.955, 3.08, Rotation2d.fromDegrees(30.538), Rotation2d.fromDegrees(-168.34))
            ), 
            new TrajectoryConfig(3.0, 3.0).setKinematics(Constants.Swerve.swerveKinematics), controllerConfig
        );


        addCommands(
            /* Set odometry and poseestimator to the startiong position, and check hood calibrated */
            AutoCommands.AutoInit(s_Swerve, tarmacToWall),
            
            /* Deploy Intake */
            new InstantCommand(() -> States.deployIntake()),
      
            
            /** GET BALL1 AND SHOOTS BALL1 AND PRE-LOADED BALL */
      
            /* Drive to wall and pickup ball */
            new InstantCommand(() -> States.intake()),
            new SwerveController(tarmacToWall),
      
            /* Activate Shooter and shoot 2 balls */
            new InstantCommand(() -> States.activateShooter()),
            new WaitCommand(0.8), 
            
            new ParallelDeadlineGroup(
                new WaitCommand(0.8),
                new InstantCommand(() -> States.feed())
            ),
      
            new InstantCommand(() -> States.stopIntake()),
            new InstantCommand(() -> States.deactivateShooter()),
      
            
            /** GET BALL2 AND SHOOTS BALL2 */
      
            /* Picks up ball 2 while doing the second trajectory */
            new InstantCommand(() -> States.intake()),
            new SwerveController(wallToCenter),
      
            /* Activates the shooter and shoots the 1 ball */
            new InstantCommand(() -> States.activateShooter()),
            new WaitCommand(0.8), 
            
            new ParallelDeadlineGroup(
                new WaitCommand(0.8),
                new InstantCommand(() -> States.feed())
            ),
      
            new InstantCommand(() -> States.stopIntake()),
            new InstantCommand(() -> States.deactivateShooter()),
      
            
            /** GET BALL3 AND BALL4 AND SHOOT BALL3 AND BALL4 */
      
            /* Pick up 2 balls (ball3 and ball4) while doing the third trajectory */
            new InstantCommand(() -> States.intake()),
            new SwerveController(centerToHpStation),
      
            /* Wait in front of the terminal for the human player ball */
            new WaitCommand(0.6),

            /* Drive to Center for Shooting Position */
            new SwerveController(hpStationToCenter),
      
            /* Activate the shooter and shoot the 2 balls */
            new InstantCommand(() -> States.activateShooter()),
            new WaitCommand(0.8), 
            
            new ParallelDeadlineGroup(
                new WaitCommand(0.8),
                new InstantCommand(() -> States.feed())
            ),
      
            new InstantCommand(() -> States.stopIntake()),
            new InstantCommand(() -> States.deactivateShooter())
        );
    }
}
