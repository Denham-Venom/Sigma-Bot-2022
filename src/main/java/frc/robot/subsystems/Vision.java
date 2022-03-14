// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.lib.util.Limelight.ledStates;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;

public class Vision extends SubsystemBase {
  private Limelight limelight;
  private ShooterStates shooterState = States.shooterState;
  /** Creates a new Vision. */
  public Vision() {
    limelight = new Limelight(
      Constants.Vision.limelightHeight,
      Constants.Vision.limelightAngle,
      Constants.Vision.goalHeight
    );
  }

  public Limelight getLimelight() {
    return this.limelight;
  }

  @Override
  public void periodic() {
    if(States.shooterState != this.shooterState) {
      this.shooterState = States.shooterState;
      if (States.shooterState == ShooterStates.preShoot || Constants.Shooter.calibrationMode){
        limelight.ledState(ledStates.on);
      }
      else{
        limelight.ledState(ledStates.off);
      }
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LLDistance",limelight.getDistance().getNorm());
  }
}
