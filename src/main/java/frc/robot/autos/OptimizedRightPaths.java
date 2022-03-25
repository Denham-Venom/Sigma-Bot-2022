// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.SwerveTrajectory;
import frc.lib.util.SwerveTrajectoryWaypoint;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class OptimizedRightPaths extends SequentialCommandGroup {

  //private Timer m_timer;
  private int waypointIndex;

  /** Creates a new OptimizedRightPaths. */
  public OptimizedRightPaths(Swerve s_Swerve) {

    waypointIndex = 0;
    SwerveTrajectoryWaypoint startPos = AutoCommands.getStartingPose("RightOptimized");

    SwerveTrajectory opRightPaths1 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig, 
      startPos, //start
      AutoConstants.optimizedRightPoints [waypointIndex++] //ball1 & shoot
    );
    
    SwerveTrajectory opRightPaths2 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.optimizedRightPoints [waypointIndex++], //start from ball1
      AutoConstants.optimizedRightPoints [waypointIndex++] //go to ball2 and shoot
    );

    SwerveTrajectory opRightPaths3 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.optimizedRightPoints [waypointIndex++], //start from ball2
      AutoConstants.optimizedRightPoints [waypointIndex++] //go to get ball 3 and 4
    );

    SwerveTrajectory opRightPaths4 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.optimizedRightPoints [waypointIndex++], //go to get ball 3 and 4
      AutoConstants.optimizedRightPoints [waypointIndex] //go to shoot
    );
    
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.thetaKP, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand1 = 
        new SwerveControllerCommand(
            opRightPaths1.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.xKP, 0, 0),
            new PIDController(Constants.Swerve.yKP, 0, 0),
            thetaController,
            opRightPaths1.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand2 = 
        new SwerveControllerCommand(
            opRightPaths2.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.xKP, 0, 0),
            new PIDController(Constants.Swerve.yKP, 0, 0),
            thetaController,
            opRightPaths2.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand3 = 
        new SwerveControllerCommand(
            opRightPaths3.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.xKP, 0, 0),
            new PIDController(Constants.Swerve.yKP, 0, 0),
            thetaController,
            opRightPaths3.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand4 = 
        new SwerveControllerCommand(
            opRightPaths4.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.xKP, 0, 0),
            new PIDController(Constants.Swerve.yKP, 0, 0),
            thetaController,
            opRightPaths4.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
      // GETS BALL1 AND SHOOTS BALL1 AND THE PRE-LOADED BALL
      
      //Gets the initial pose
      new InstantCommand(() -> s_Swerve.resetOdometry(startPos.getPositionAndOrientation())),
      //Deploys the intake
      new InstantCommand(() -> States.deployIntake()),

      //Picks up ball 1 (ball1)
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand1,

      //Activates the shooter and shoots the 2 balls
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(1.0), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter()),

      //GETS BALL2 AND SHOOTS BALL2

      //picks up 1 ball (ball2)
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand2,

      //Activates the shooter and shoots the 1 ball
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(1.0), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter()),

       //GETS BALL3 AND BALL4 AND SHOOTS BALL3 AND BALL4

      //picks up 2 ball (ball3 and ball4)
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand3,

      swerveControllerCommand4,

      //Activates the shooter and shoots the 1 ball
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(1.0), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter())
    );
    new InstantCommand(() -> s_Swerve.resetOdometry(AutoConstants.optimizedRightPoints [waypointIndex-1].getPositionAndOrientation()));

  };
}
    