// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazyTalonFX;
import frc.robot.Constants;

public class Turreter extends SubsystemBase {
  /** Creates a new Turret. */
  public LazyTalonFX hoodMotor;
  public LazyTalonFX turretMotor;
  private PIDController hoodController;
  private PIDController turretController;
  private DutyCycleEncoder hoodEncoder;
  private DutyCycleEncoder turretEncoder;
  public Turreter() {
    hoodMotor = new LazyTalonFX(Constants.Turret.turretMotor1Constants);
    turretMotor = new LazyTalonFX(Constants.Turret.turretMotor2Constants);
  }

  public double getHoodAngle(){
    double position = hoodEncoder.get() * 360;
    position = 360 - position;
    position = position - Constants.Turret.hoodEncoderOffset;
    position = Math.round(position * 100.0) / 100.0; // Rounding to 2 decimal places
    return position;
  }

  public void setHoodAngle(double angle){
    double finalAngle = angle;
    if (angle < Constants.Turret.hoodReverseLimit){
        finalAngle = Constants.Turret.hoodReverseLimit;
    }
    else if (angle > Constants.Turret.hoodForwardLimit) {
        finalAngle = Constants.Turret.hoodForwardLimit;
    }
    double demand = hoodController.calculate(getHoodAngle(), finalAngle);
    hoodMotor.set(ControlMode.PercentOutput, demand);
  }

  public double getTurretAngle(){
    double position = turretEncoder.get() * 360;
    position = 360 - position;
    position = position - Constants.Turret.turretEncoderOffset;
    position = Math.round(position * 100.0) / 100.0; // Rounding to 2 decimal places
    return position;
  }

  public void setTurretAngle(double angle){
    double finalAngle = angle;
    if (angle < Constants.Turret.turretReverseLimit){
        finalAngle = Constants.Turret.turretReverseLimit;
    }
    else if (angle > Constants.Turret.turretForwardLimit) {
        finalAngle = Constants.Turret.turretForwardLimit;
    }
    double demand = turretController.calculate(getTurretAngle(), finalAngle);
    turretMotor.set(ControlMode.PercentOutput, demand);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
