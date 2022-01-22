// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Limelight;
import frc.lib.util.Limelight.ledStates;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;

public class Vision extends SubsystemBase {
  public Limelight limelight;
  /** Creates a new Vision. */
  public Vision() {
    limelight = new Limelight(
      Constants.Vision.limelightHeight,
      Constants.Vision.limelightAngle,
      Constants.Vision.goalHeight
    );
  }

  @Override
  public void periodic() {
    if (States.shooterState == ShooterStates.preShoot || Constants.Shooter.calibrationMode){
      limelight.ledState(ledStates.on);
    }
    else{
      limelight.ledState(ledStates.off);
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LLDistance",limelight.getDistance().getNorm());
  }
}
