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

  /** Creates a new rightTarmacPaths. */
  public RightTarmacPaths(Swerve s_Swerve, String position, int numBalls) {

    waypointIndex = 0;
    SwerveTrajectoryWaypoint startPos = AutoCommands.getStartingPose("Right" + position);


    SwerveTrajectory rightTarmacPaths1 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig, 
      startPos, //start
      AutoConstants.rightPoints [waypointIndex++] //ball1 & shoot
    );
    
    SwerveTrajectory rightTarmacPaths2 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.rightPoints [waypointIndex++], //start from ball1
      AutoConstants.rightPoints [waypointIndex++], //ball2
      AutoConstants.rightPoints [waypointIndex++], //ball3
      AutoConstants.rightPoints [waypointIndex++] //shoot pos
    );
    
    SwerveTrajectory rightTarmacPaths3 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.rightPoints [waypointIndex++], //start shoot pos
      AutoConstants.rightPoints [waypointIndex++], //ball4
      AutoConstants.rightPoints [waypointIndex++] //end final shoot pos
    );

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.thetaKP, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand1 = 
        new SwerveControllerCommand(
            rightTarmacPaths1.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.xKP, 0, 0),
            new PIDController(Constants.Swerve.yKP, 0, 0),
            thetaController,
            rightTarmacPaths1.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand2 = 
        new SwerveControllerCommand(
            rightTarmacPaths2.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.xKP, 0, 0),
            new PIDController(Constants.Swerve.yKP, 0, 0),
            thetaController,
            rightTarmacPaths2.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand swerveControllerCommand3 = 
        new SwerveControllerCommand(
            rightTarmacPaths3.getTrajectory(),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.xKP, 0, 0),
            new PIDController(Constants.Swerve.yKP, 0, 0),
            thetaController,
            rightTarmacPaths3.getAngleSupplier(),
            s_Swerve::setModuleStates,
            s_Swerve);
            
    addCommands(
      //Gets the initial pose
      new InstantCommand(() -> s_Swerve.resetOdometry(rightTarmacPaths1.getInitialPose())),
      //Deploys the intake
      new InstantCommand(() -> States.deployIntake()),

      //Picks up ball 1  
      new InstantCommand(() -> States.intake()),
      swerveControllerCommand1,
      new InstantCommand(() -> States.stopIntake()),

      //Activates the shooter and shoots the 2 balls
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(1.0), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.stopIntake()),
      new InstantCommand(() -> States.deactivateShooter())
    );
    if(numBalls == 4 || numBalls == 5) {
      addCommands(
      //Picks up ball 2 and 3
      new InstantCommand(() -> {
        States.deployIntake();
        States.intake();
      }),
      swerveControllerCommand2,
    
      new InstantCommand(() -> States.stopIntake()),

      //Activates the shooter and shoots the 2 balls
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(1.0), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.deactivateShooter()),
      new InstantCommand(() -> States.stopIntake())
      );
    }
      if(numBalls == 5) {
      addCommands(
      //Picks up ball 4
      new InstantCommand(() -> {
        States.deployIntake();
        States.intake();
      }),
      swerveControllerCommand3,

      //Activated the shooter and shoots the ball
      new InstantCommand(() -> States.activateShooter()),
      new WaitCommand(1.0), 
      
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new InstantCommand(() -> States.feed())),

      new InstantCommand(() -> States.deactivateShooter()),
      new InstantCommand(() -> States.stopIntake())
      );
    }
  }
}

//InstantCommand starts intaking
//path 1
//path 2
//stop intaking
//preshoot
//feed
//stop shooter
//stop intake


// public class enum Pose2d[] {
//    case: blah
//       code;
//    case: blahblah
//       code;
//    return(code);
// }