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

public class RightTarmacPaths extends SequentialCommandGroup {

  //private Timer m_timer;
  private int waypointIndex;

  /** Creates a new OptimizedRightPaths. */
  public RightTarmacPaths(Swerve s_Swerve) {

    waypointIndex = 0;
    SwerveTrajectoryWaypoint startPos = AutoCommands.getStartingPose("Right");

    SwerveTrajectory opRightPaths1 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig, 
      startPos, //start
      AutoConstants.optimizedRightPoints [waypointIndex++] //ball 1 & shoot
    );
    
    SwerveTrajectory opRightPaths2 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.optimizedRightPoints [waypointIndex++], //start from ball 1
      AutoConstants.optimizedRightPoints [waypointIndex++] //go to ball 2 and shoot
    );

    SwerveTrajectory opRightPaths3 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.optimizedRightPoints [waypointIndex++], //start from ball 2
      AutoConstants.optimizedRightPoints [waypointIndex++] //go to get ball 3 and 4
    );

    SwerveTrajectory opRightPaths4 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.optimizedRightPoints [waypointIndex++], //go to get ball 3 and 4
      AutoConstants.optimizedRightPoints [waypointIndex] //go to shoot
    );

    var xController = new PIDController(Constants.Swerve.xKP, 0, 0);
    var yController = new PIDController(Constants.Swerve.yKP, 0, 0);
    
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.thetaKP, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand1 = 
        new SwerveControllerCommand(
            opRightPaths1.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            xController,
            yController,
            thetaController,
            opRightPaths1.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand2 = 
        new SwerveControllerCommand(
            opRightPaths2.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            xController,
            yController,
            thetaController,
            opRightPaths2.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand3 = 
        new SwerveControllerCommand(
            opRightPaths3.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            xController,
            yController,
            thetaController,
            opRightPaths3.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand4 = 
        new SwerveControllerCommand(
            opRightPaths4.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            xController,
            yController,
            thetaController,
            opRightPaths4.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
      //Gets the initial pose
      new InstantCommand(() -> s_Swerve.resetOdometry(startPos.getPositionAndOrientation())),
      //Deploys the intake
      new InstantCommand(() -> States.deployIntake()),

      //GETS BALL1 AND SHOOTS BALL1 AND PRE-LOADED BALL

      //Picks up 1 ball (ball1) while doing the first trajectory
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand1,

      //Activates the shooter and shoots the 2 balls
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(0.8), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(0.8),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter()),

      //GETS BALL2 AND SHOOTS BALL2

      //picks up 1 ball (ball2) while doing the second trajectory
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand2,

      //Activates the shooter and shoots the 1 ball
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(0.8), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(0.8),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter()),

       //GETS BALL3 AND BALL4 AND SHOOTS BALL3 AND BALL4

      //picks up 2 balls (ball3 and ball4) while doing the third trajectory
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand3,

      //waits in front of the terminal for the human player ball
      new WaitCommand(0.6),
      swerveControllerCommand4,

      //Activates the shooter and shoots the 2 ball
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(0.8), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(0.8),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter())
    );
    //new InstantCommand(() -> s_Swerve.resetOdometry(AutoConstants.optimizedRightPoints [waypointIndex-1].getPositionAndOrientation()));

  };
}
    