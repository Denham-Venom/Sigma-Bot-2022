// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazyTalonFX;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private LazyTalonFX shooterMotorParent;
  private LazyTalonFX shooterMotorChild;
  public LazyTalonFX hoodMotor;
  public LazyTalonFX turretMotor;
  
  public Shooter() {
    shooterMotorParent = new LazyTalonFX(Constants.Shooter.rotateShooterConstants);
    shooterMotorChild = new LazyTalonFX(Constants.Shooter.kickerShooterConstants);
    shooterMotorParent.configPID(Constants.Shooter.shooterPID);
    shooterMotorChild.follow(shooterMotorParent);
  }

  public double getShooterRPM(){
    return Conversions.falconToRPM(shooterMotorParent.getSelectedSensorVelocity(), Constants.Shooter.shooterGearRatio);
  }

  public void setShooterRPM(double shooterRPM){
    double falconVelocity = Conversions.RPMToFalcon(shooterRPM, Constants.Shooter.shooterGearRatio);
    shooterMotorParent.set(ControlMode.Velocity, falconVelocity);
  }

  //getter and setter for the hood
  public double getHoodAngle(){
    return Conversions.falconToDegrees(hoodMotor.getSelectedSensorPosition(), Constants.Shooter.hoodGearRatio);
  }

  public void setHoodAngle(double hoodAngle){
    if (hoodAngle > Constants.Shooter.hoodHighLimit){
      hoodAngle = Constants.Shooter.hoodHighLimit;
    }
    else if (hoodAngle < Constants.Shooter.hoodLowLimit){
      hoodAngle = Constants.Shooter.hoodLowLimit;
    }
    hoodMotor.set(ControlMode.Position, Conversions.degreesToFalcon(hoodAngle, Constants.Shooter.hoodGearRatio));
  }
    

  //getter and setter for the turret
  public double getTurretAngle(){
    return Conversions.falconToDegrees(turretMotor.getSelectedSensorPosition(), Constants.Shooter.turretGearRatio);
  }
  
  /**
   * 
   * @param turretAngle Angle between -180 to 180
   */
  public void setTurretAngle(double turretAngle){
    if(turretAngle < 0) {
      turretAngle += 360;
    }

    double finalAngle = Conversions.degreesToFalcon(turretAngle, Constants.Shooter.turretGearRatio);
    if (finalAngle > Constants.Shooter.turretHighLimit){
      finalAngle = Constants.Shooter.turretHighLimit;
    }
    else if (finalAngle < Constants.Shooter.turretLowLimit){
      finalAngle = Constants.Shooter.turretLowLimit;
    }
    turretMotor.set(ControlMode.Position, finalAngle);
  }


  public void setPower(double power){
    shooterMotorParent.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
