// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.lib.util.Limelight.ledStates;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;


public class Vision extends SubsystemBase {
  private Limelight limelight;
  ShuffleboardTab Drivers = Shuffleboard.getTab("Drivers");
  NetworkTableEntry LLDist = Drivers.add("LLDist", 0).getEntry();
  private ShooterStates shooterState = States.shooterState;
  /** Creates a new Vision. */
  public Vision() {
    limelight = new Limelight(
      Constants.Vision.limelightHeight,
      Constants.Vision.limelightAngle,
      Constants.Vision.goalHeight
    );
    limelight.ledState(ledStates.on);
  }

  public Limelight getLimelight() {
    return this.limelight;
  }

  @Override
  public void periodic() {
    if(States.shooterState != this.shooterState) {
      this.shooterState = States.shooterState;
      if (States.shooterState == ShooterStates.preShoot){
        limelight.ledState(ledStates.on);
      }
      else{
        limelight.ledState(ledStates.on);
      }
    }
    if(Constants.Shooter.calibrationMode) limelight.ledState(ledStates.on);
    // This method will be called once per scheduler run
    LLDist.setDouble(limelight.getDistance().getNorm());
  }
}
