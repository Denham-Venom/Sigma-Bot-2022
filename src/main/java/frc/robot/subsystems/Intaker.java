// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazySparkMAX;
import frc.Controllers.LazyTalonFX;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.IntakeExtendStates;
import frc.robot.States.IntakeStates;

public class Intaker extends SubsystemBase {
  /** Creates a new Intaker. */
 
  // indexer 1 by intake
  private LazyTalonFX indexerMotor;
  // indexer 2 by shooter
  private LazySparkMAX spinUpMotor;
  // intaker
  private LazyTalonFX intakeMotor;
  
  private DoubleSolenoid intakeExtend;
  private DigitalInput intakeSensor;
  private DigitalInput shooterSensor;
  public Intaker(PneumaticHub m_pHub) {
    indexerMotor = new LazyTalonFX(Constants.Intake.intakeMotorConstants);
    //spinUpMotor = new LazySparkMAX(Constants.Intake.spinUpMotorConstants);
    intakeMotor = new LazyTalonFX(Constants.Intake.intakeMotorConstants);
    intakeExtend = m_pHub.makeDoubleSolenoid(Constants.Intake.IntakeSolenoidForwardChannel, Constants.Intake.IntakeSolenoidReverseChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(States.intakeState) {
      case intaking:
      // check if a ball is detected by a sensor (2 total)
      // if it is, run only the motors with no ball detected
        if(!intakeSensor.get()) {
          if(!shooterSensor.get()) {
            //spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.IntakeSpeed);
          } else {
            spinUpMotor.set(ControlType.kDutyCycle, 0);
          }
          indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
          intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        }
        else if(!shooterSensor.get()) {
          indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
          //spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.IntakeSpeed);
          intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        } else {
          indexerMotor.set(ControlMode.PercentOutput, 0);
          //spinUpMotor.set(ControlType.kDutyCycle, 0);
          intakeMotor.set(ControlMode.PercentOutput, 0);
        }
        break;
      case outtaking:
      // reverses the intake motor
        intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.IntakeSpeed);
        break;
      case feeding: 
      // Runs all intake/indexer motors
        indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        //spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.IntakeSpeed);
        intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        break;
      case disabled:
      // Stops all intake/indexer motors
        indexerMotor.set(ControlMode.PercentOutput, 0);
        //spinUpMotor.set(ControlType.kDutyCycle, 0);
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    switch(States.intakeExtendState) {
      case deployIntake:
        intakeExtend.set(Value.kForward);
        break;
      case retractIntake:
        intakeExtend.set(Value.kReverse); 
        break;
      case disabled:
        intakeExtend.set(Value.kOff);
      }
      
    }
  }

