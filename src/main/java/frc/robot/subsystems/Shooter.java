// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.LazyTalonFX;
import frc.lib.Controllers.LazyTalonSRX;
import frc.lib.math.Conversions;
import frc.lib.math.PIDGains;
import frc.lib.util.Interpolatable;
import frc.lib.util.InterpolatableTreeMap;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;
//import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Swerve;
import frc.robot.States.ShooterStates;

public class Shooter extends SubsystemBase {

  /** Creates a new Shooter. */
  private LazyTalonFX shooterMotorParent;
  private LazyTalonFX shooterMotorChild;
  private LazyTalonSRX hoodMotor;
  //private LazyTalonFX turretMotor;
  private Limelight limelight;
  private boolean homingDone = false;
  private DigitalInput hoodLimit = new DigitalInput(Constants.Shooter.hoodLimitSwitchID);
  // private RelativeEncoder hoodEncoder;
  // private Consumer<RelativeEncoder> encoderGetter = (RelativeEncoder encoder) -> {
  //   this.hoodEncoder = encoder;
  //   this.hoodEncoder.setPositionConversionFactor(Constants.Shooter.hoodGearRatio * 360 /*degrees*/);
  // };
  private double targetShooterRPM = 0;
  private DutyCycleEncoder hoodEncoderAbsolute = new DutyCycleEncoder(new DigitalInput(Constants.Shooter.hoodEncoderAbsoluteChannel));
  private Encoder hoodEncoder = new Encoder(1, 2, false, Encoder.EncodingType.k4X);
  private InterpolatableTreeMap<Double> shooterMap = new InterpolatableTreeMap<>();
  private InterpolatableTreeMap<Double> hoodMap = new InterpolatableTreeMap<>();
  private ShuffleboardTab testing = Shuffleboard.getTab("Testing");
  private ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
  private ShuffleboardTab drivers = Shuffleboard.getTab("Drivers");
  private NetworkTableEntry shootP = tuning.add("Shoot P", 0).getEntry();
  private NetworkTableEntry shootI = tuning.add("Shoot I", 0).getEntry();
  private NetworkTableEntry shootD = tuning.add("Shoot D", 0).getEntry();
  private NetworkTableEntry shootRPM = tuning.add("Shoot RPM", 0.).getEntry();
  private NetworkTableEntry setShootRPM = tuning.add("Set Shoot RPM", 0.).getEntry();
  private NetworkTableEntry tuneShoot = tuning.add("Tune Shoot", false).getEntry();
  private PIDGains tuningShooterPID;
  private NetworkTableEntry hoodP = tuning.add("Hood P", 0).getEntry();
  private NetworkTableEntry hoodI = tuning.add("Hood I", 0).getEntry();
  private NetworkTableEntry hoodD = tuning.add("Hood D", 0).getEntry();
  private NetworkTableEntry hoodAng = tuning.add("Hood Angle", 0.).getEntry();
  private NetworkTableEntry setHoodAng = tuning.add("Set Hood Angle", 0.).getEntry();
  private NetworkTableEntry tuneHood = tuning.add("Tune Hood", false).getEntry();
  private NetworkTableEntry hoodPIDOut = tuning.add("HoodPIDOut", 0.).getEntry();
  private NetworkTableEntry hoodLimitSwitchPressed = tuning.add("HoodLimitPressed", false).getEntry();
  private NetworkTableEntry hoodReady = drivers.add("Hood Ready", false).getEntry();
  private boolean shooterReady = false;
  private NetworkTableEntry getDistEntry = drivers.add("GetDist", 0).getEntry();
  private PIDController hoodController;
  private ShooterStates state;
  private Swerve swerve;
  

  public Shooter(Vision m_Vision, Swerve m_Swerve) {
    shooterMotorParent = new LazyTalonFX(Constants.Shooter.parentShooterConstants);
    shooterMotorChild = new LazyTalonFX(Constants.Shooter.childShooterConstants);
    hoodMotor = new LazyTalonSRX(Constants.Shooter.hoodConstants);
    //turretMotor = new LazyTalonFX(Constants.Shooter.turretConstants);
    shooterMotorChild.follow(shooterMotorParent);
    //hoodMotor.configPID(Constants.Shooter.hoodPID);
    //turretMotor.configPID(Constants.Shooter.turretPID);
    limelight = m_Vision.getLimelight();

    this.swerve = m_Swerve;

    tuningShooterPID = Constants.Shooter.shooterPID;
    hoodController = new PIDController(
      Constants.Shooter.hoodPID.kP, 
      Constants.Shooter.hoodPID.kI, 
      Constants.Shooter.hoodPID.kD
    );
    hoodController.setTolerance(Constants.Shooter.hoodControllerToleranceDegrees, 0.5);
    hoodEncoder.reset();
    hoodEncoder.setDistancePerPulse(360. * Constants.Shooter.hoodGearRatio / 2048.); //degrees

    testing.add("Start Shooter Motors", new InstantCommand(
      () -> setPower(0.5)
    ));

    testing.add("Stop Shooter Motors", new InstantCommand(
      () -> setPower(0)
    ));

    testing.add("UP Hood Motor", new InstantCommand(
      () -> hoodMotor.set (ControlMode.PercentOutput, 0.15)
    ));

    testing.add("DOWN Hood Motor", new InstantCommand(
      () -> hoodMotor.set (ControlMode.PercentOutput, -0.15)
    ));

    testing.add("STOP Hood Motor", new InstantCommand(
      () -> hoodMotor.set (ControlMode.PercentOutput, 0)
    ));


    if (Constants.Shooter.calibrationMode){
      //tuning.add("Shooter RPM Calib", 0);
      //tuning.add("Shooter Angle Calib", 0);
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
    targetShooterRPM = shooterRPM;
    double falconVelocity = Conversions.RPMToFalcon(shooterRPM, Constants.Shooter.shooterGearRatio);
    shooterMotorParent.set(ControlMode.Velocity, falconVelocity);//, DemandType.ArbitraryFeedForward, Constants.Shooter.shooterFF.calculate(falconVelocity));
  }

  public void resetHood() {
    homingDone = false;
  }

  // public Consumer<RelativeEncoder> getHoodEncoderConsumer() {
  //   return this.encoderGetter;
  // }

  //getter and setter for the hood
  // public double getHoodAngle(){
  //   if(hoodEncoder == null) return Double.MIN_VALUE;
  //   return hoodEncoder.getPosition();
  //   return hoodEncoderAbsolute.getAbsolutePosition();
  // }

  public double getHoodAngle(){
    return hoodEncoder.getDistance() + Constants.Shooter.hoodAngleOffset;
    // double position = 360 - hoodEncoderAbsolute.get() * 360;
    // if(position >= Constants.Shooter.hoodEncoderOffset)
    //   position -= Constants.Shooter.hoodEncoderOffset;
    // else
    //   position = position + (360 - Constants.Shooter.hoodEncoderOffset);

    // //position = 360 - position;
    // //position = position - Constants.Shooter.hoodEncoderOffset;
    // position = Math.round(position * 100.0) / 100.0; // Rounding to 2 decimal places
    // return position;
  }

  public void setHoodAngle(double hoodAngle){
    if (hoodAngle > Constants.Shooter.hoodHighLimit){
      hoodAngle = Constants.Shooter.hoodHighLimit;
    }
    else if (hoodAngle < Constants.Shooter.hoodLowLimit){
      hoodAngle = Constants.Shooter.hoodLowLimit;
    }
    hoodController.setSetpoint(hoodAngle);
    if(hoodController.atSetpoint()) {
      hoodMotor.set(ControlMode.PercentOutput, 0);
      hoodReady.setBoolean(true);
      return;
    }
    hoodReady.setBoolean(false);
      
    double output = hoodController.calculate(getHoodAngle());// + Constants.Shooter.hoodPID.kFF;
    if(output < 0) {
      if(!hoodLimit.get()) output = 0;
      else output += Constants.Shooter.hoodDownFF;
    } else if(output > 0) {
      output += Constants.Shooter.hoodPID.kFF;
    }
    hoodPIDOut.setDouble(output);
    hoodMotor.set(ControlMode.PercentOutput, output);//Conversions.degreesToFalcon(hoodAngle, Constants.Shooter.hoodGearRatio));
  }

  public void setHoodPower(double scale) {
    hoodMotor.set(ControlMode.PercentOutput, scale * 0.2);//SmartDashboard.getNumber("HoodTestPow", 0));
  }

  public void stopHood() {
    hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isShooterReady() {
    if(getShooterRPM() >= targetShooterRPM - Constants.Shooter.tolerance && getShooterRPM() <= targetShooterRPM + Constants.Shooter.tolerance) {
      return true;
    }
    return false;
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
    getDistEntry.setDouble(limelight.getDistance().plus(new Translation2d(Constants.Vision.goalDiameter/2, 0)).getNorm());
    
    boolean hoodDown = !hoodLimit.get();
    hoodLimitSwitchPressed.setBoolean(hoodDown);
    if(hoodDown) hoodEncoder.reset();

    if(!homingDone) {
      if(hoodDown) homingDone = true;
      setHoodPower(-1);
      return;
    }
    // if(Climber.canClimb()) {
    //   return;
    // }
    //if(States.shooterState != state) {
    //  state = States.shooterState;
      switch(States.shooterState){
        case disabled:
          shooterMotorParent.set(ControlMode.PercentOutput, 0); 
          if(!Constants.tuningMode) {
            setHoodAngle(10);
            //turretMotor.set(ControlMode.PercentOutput, 0);
          }
          break;
            
        case preShoot:
          if (Constants.Shooter.calibrationMode){
              setShooterRPM(setShootRPM.getDouble(0));
              setHoodAngle(setHoodAng.getDouble(0));
          } else{
              // setShooterRPM(shooterMap.get(limelight.getDistance().getNorm()));
              // setHoodAngle(hoodMap.get(limelight.getDistance().getNorm()));
              setShooterRPM(shooterMap.get(limelight.getDistance().plus(new Translation2d(Constants.Vision.goalDiameter/2, 0)).getNorm()));
              setHoodAngle(hoodMap.get(limelight.getDistance().plus(new Translation2d(Constants.Vision.goalDiameter/2, 0)).getNorm()));
          }
          break;
      //}
        case lowPreShoot:
          if(Constants.Shooter.calibrationMode) {
            setShooterRPM(setShootRPM.getDouble(0));
            setHoodAngle(setHoodAng.getDouble(0));
          } else {
            setShooterRPM(Constants.Shooter.shooterLowMap[1]);
            setHoodAngle(Constants.Shooter.shooterLowMap[2]);
          }
          break;
        default:
          break;
    }
    
    if(Constants.tuningMode || Constants.Shooter.calibrationMode) {
      shootRPM.setDouble(this.getShooterRPM());
      hoodAng.setDouble(this.getHoodAngle());
    }
    if(Constants.tuningMode) {
      double p, i, d;
      if(tuneShoot.getBoolean(false)) {
        p = shootP.getDouble(0); i = shootI.getDouble(0); d = shootD.getDouble(0);
        if(p != tuningShooterPID.kP
            || i != tuningShooterPID.kI
            || d != tuningShooterPID.kD) {
          tuningShooterPID = new PIDGains(p, i, d, tuningShooterPID.kFF);
          shooterMotorParent.configPID(tuningShooterPID);
        }
        setShooterRPM(setShootRPM.getDouble(0));
      }

      if(tuneHood.getBoolean(false)) {
        p = hoodP.getDouble(0); i = hoodI.getDouble(0); d = hoodD.getDouble(0);
        if(p != hoodController.getP()
            || i != hoodController.getI()
            || d != hoodController.getD()) {
          hoodController.setPID(p, i, d);
        }
        setHoodAngle(setHoodAng.getDouble(0));
      }
    }

    //updateTurret();
    // This method will be called once per scheduler run
  }
}
