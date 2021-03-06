// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.fasterxml.jackson.databind.node.IntNode;

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

public class LeftTarmacPaths extends SequentialCommandGroup {

  private int waypointIndex;

  /** Creates a new leftTarmacPaths. */
  public LeftTarmacPaths(Swerve s_Swerve, SwerveTrajectoryWaypoint startPos, int numBalls) {    
    
    waypointIndex = 0;

    SwerveTrajectory leftTarmacPaths1 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      startPos, //start
      AutoConstants.leftPoints [waypointIndex++]  //goes to our ball and shoots
    );

    SwerveTrajectory leftTarmacPaths2 = new SwerveTrajectory(
      Constants.AutoConstants.trajectoryConfig,
      AutoConstants.leftPoints [waypointIndex++], //goes to our ball and shoots
      AutoConstants.leftPoints [waypointIndex++], //goes to the far red ball 
      AutoConstants.leftPoints [waypointIndex++], //goes to the closer red ball
      AutoConstants.leftPoints [waypointIndex]  //goes to the hangar and desposes the ball
    );

    var thetaController =
        new ProfiledPIDController(
          Constants.AutoConstants.thetaKP, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = 
      new SwerveControllerCommand(
          leftTarmacPaths1.getTrajectory(),
          s_Swerve::getPose,
          Constants.Swerve.swerveKinematics,
          new PIDController(Constants.Swerve.xKP, 0, 0),
          new PIDController(Constants.Swerve.yKP, 0, 0),
          thetaController,
          leftTarmacPaths1.getAngleSupplier(),
          s_Swerve::setModuleStates,
          s_Swerve);

       SwerveControllerCommand swerveControllerCommand2 = 
      new SwerveControllerCommand(
          leftTarmacPaths2.getTrajectory(),
          s_Swerve::getPose,
          Constants.Swerve.swerveKinematics,
          new PIDController(Constants.Swerve.xKP, 0, 0),
          new PIDController(Constants.Swerve.yKP, 0, 0),
          thetaController,
          leftTarmacPaths2.getAngleSupplier(),
          s_Swerve::setModuleStates,
          s_Swerve);

      addCommands(
        //Gets the initial pose
        new InstantCommand(() -> s_Swerve.resetOdometry(startPos.getPositionAndOrientation()))
      );

      if(numBalls > 0) {
        addCommands(
        //Deploys the intake
        new InstantCommand(() -> States.deployIntake()),

        //GETS BALL1 AND SHOOTS BALL1 AND THE PRE-LOADED BALL

        //Does the first trajectory while intaking and picks one ball (our color)
        new InstantCommand(() -> States.intake()),
        swerveControllerCommand,

        new InstantCommand(() -> States.stopIntake()),

        //Activates the shooter and shoots the 2 balls
        new InstantCommand(() -> States.activateShooter()),
        new WaitCommand(1.0), 

        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new InstantCommand(() -> States.feed())),

        //Stops shooting
        new InstantCommand(() -> States.stopIntake()),
        new InstantCommand(() -> States.deactivateShooter())//,

        // new InstantCommand(() -> States.intake()),

        // swerveControllerCommand2,

        // new InstantCommand(() -> States.outtake()),
        // new WaitCommand(.3),

        // new InstantCommand(() -> States.stopIntake())
  
      );
      }
  }
}