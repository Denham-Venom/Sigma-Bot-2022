// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazyTalonFX;
import frc.lib.math.Conversions;
import frc.lib.math.PIDGains;
import frc.lib.util.Interpolatable;
import frc.lib.util.InterpolatableTreeMap;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private LazyTalonFX shooterMotorParent;
  private LazyTalonFX shooterMotorChild;
  private LazyTalonFX hoodMotor;
  //private LazyTalonFX turretMotor;
  private Limelight limelight;

  private InterpolatableTreeMap<Double> shooterMap = new InterpolatableTreeMap<>();
  private InterpolatableTreeMap<Double> hoodMap = new InterpolatableTreeMap<>();
  private ShuffleboardTab testing = Shuffleboard.getTab("Testing");
  private ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
  private NetworkTableEntry shootP = tuning.add("Shoot P", 0).getEntry();
  private NetworkTableEntry shootI = tuning.add("Shoot I", 0).getEntry();
  private NetworkTableEntry shootD = tuning.add("Shoot D", 0).getEntry();
  private NetworkTableEntry hoodP = tuning.add("Hood P", 0).getEntry();
  private NetworkTableEntry hoodI = tuning.add("Hood I", 0).getEntry();
  private NetworkTableEntry hoodD = tuning.add("Hood D", 0).getEntry();
  private PIDGains tuningShooterPID;
  private PIDGains tuningHoodPID;
  

  public Shooter(Vision m_Vision) {
    shooterMotorParent = new LazyTalonFX(Constants.Shooter.childShooterConstants);
    shooterMotorChild = new LazyTalonFX(Constants.Shooter.childShooterConstants);
    hoodMotor = new LazyTalonFX(Constants.Shooter.hoodConstants);
    //turretMotor = new LazyTalonFX(Constants.Shooter.turretConstants);

    shooterMotorParent.configPID(Constants.Shooter.shooterPID);
    shooterMotorChild.follow(shooterMotorParent);
    hoodMotor.configPID(Constants.Shooter.hoodPID);
    //turretMotor.configPID(Constants.Shooter.turretPID);
    limelight = m_Vision.getLimelight();

    tuningShooterPID = Constants.Shooter.shooterPID;
    tuningHoodPID = Constants.Shooter.hoodPID;

    testing.addNumber("Hood Angle", this::getHoodAngle);


    testing.add("Start Shooter Motors", new InstantCommand(
      () -> setPower(0.5)
    ));

    testing.add("Stop Shooter Motors", new InstantCommand(
      () -> setPower(0)
    ));

    tuning.add("Shoot P", 0.);
    tuning.add("Shoot I", 0.);
    tuning.add("Shoot D", 0.);

    if (Constants.Shooter.calibrationMode){
      SmartDashboard.getNumber("Shooter RPM Calib", 0);
      SmartDashboard.getNumber("Shooter Angle Calib", 0);
    }

    for (int i = 0; i < Constants.Shooter.shooterMap.length; ++i) {
      shooterMap.set(Constants.Shooter.shooterMap[i][0], Interpolatable.interDouble(Constants.Shooter.shooterMap[i][1]));
      hoodMap.set(Constants.Shooter.shooterMap[i][0], Interpolatable.interDouble(Constants.Shooter.shooterMap[i][2]));
    }
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
  // public Rotation2d getTurretAngle(){
  //   final double angle = Conversions.falconToDegrees(turretMotor.getSelectedSensorPosition(), Constants.Shooter.turretGearRatio);
  //   return Rotation2d.fromDegrees(angle);
  // }
  
  /**
   * 
   * @param turretAngle Angle between -180 to 180 degrees
   */
  // public void setTurretAngle(double turretAngle){
  //   if(turretAngle < 0) {
  //     turretAngle += 360;
  //   }
  //   double finalAngle = Conversions.degreesToFalcon(turretAngle, Constants.Shooter.turretGearRatio);
  //   if (finalAngle > Constants.Shooter.turretHighLimit){
  //     finalAngle = Constants.Shooter.turretHighLimit;
  //   }
  //   else if (finalAngle < Constants.Shooter.turretLowLimit){
  //     finalAngle = Constants.Shooter.turretLowLimit;
  //   }
  //   turretMotor.set(ControlMode.Position, finalAngle);
  // }


  public void setPower(double power){
    shooterMotorParent.set(ControlMode.PercentOutput, power);
  }

  // void updateTurret() {
  //   //get limelight angle
  //   Rotation2d ang = limelight.getTx();
  //   ang = getTurretAngle().plus(ang);
  //   //pass to setTurretAngle()
  //   setTurretAngle(ang.getDegrees());
  // }

  @Override
  public void periodic() {
    if(Climber.canClimb()) {
      return;
    }
    switch(States.shooterState){
      case disabled:
          shooterMotorParent.set(ControlMode.PercentOutput, 0);
          hoodMotor.set(ControlMode.PercentOutput, 0);
          //turretMotor.set(ControlMode.PercentOutput, 0);
          break;
          
      case preShoot:
          if (Constants.Shooter.calibrationMode){
              setShooterRPM(SmartDashboard.getNumber("Shooter RPM Calib", 0));
              setHoodAngle(SmartDashboard.getNumber("Shooter Angle Calib", 0));
          } else{
              setShooterRPM(shooterMap.get(limelight.getDistance().getNorm()));
              setHoodAngle(hoodMap.get(limelight.getDistance().getNorm()));
          }
          break;
    }

    if(Constants.tuningMode) {
      if(shootP.getDouble(0) != Constants.Shooter.shooterPID.kP
          | shootI.getDouble(0) != Constants.Shooter.shooterPID.kI
          | shootD.getDouble(0) != Constants.Shooter.shooterPID.kD) {
        tuningShooterPID = new PIDGains(
          shootP.getDouble(0), 
          shootI.getDouble(0), 
          shootD.getDouble(0), 
          tuningShooterPID.kFF);
        shooterMotorParent.configPID(tuningShooterPID);
      }

      if(hoodP.getDouble(0) != Constants.Shooter.hoodPID.kP
          | hoodI.getDouble(0) != Constants.Shooter.hoodPID.kI
          | hoodD.getDouble(0) != Constants.Shooter.hoodPID.kD) {
        tuningHoodPID = new PIDGains(
          hoodP.getDouble(0), 
          hoodI.getDouble(0), 
          hoodD.getDouble(0), 
          tuningHoodPID.kFF);
        hoodMotor.configPID(tuningHoodPID);
      }
    }

    //updateTurret();
    // This method will be called once per scheduler run
  }
}
