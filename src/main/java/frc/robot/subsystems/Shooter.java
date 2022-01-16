// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazyTalonFX;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private LazyTalonFX shooterMotorParent;
  private LazyTalonFX shooterMotorChild;
  
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

  public void setPower(double power){
    shooterMotorParent.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
