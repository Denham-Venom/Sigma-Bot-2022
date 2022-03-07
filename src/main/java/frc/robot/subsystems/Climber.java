// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazySparkMAX;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.States;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private LazySparkMAX climberMotor;
  private DoubleSolenoid climberPiston;
  private RelativeEncoder positionEncoder;
  private final ShuffleboardTab testing;
  public Climber(PneumaticHub m_pHub) {
    testing = Shuffleboard.getTab("Testing");
    climberMotor = new LazySparkMAX(Constants.Climber.climberMotorConstants);
    climberPiston = m_pHub.makeDoubleSolenoid(Constants.Climber.ClimberSolenoidForwardChannel, Constants.Climber.ClimberSolenoidReverseChannel);
    positionEncoder = climberMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);// Rev throughbore encoder //new DutyCycleEncoder(new DigitalInput(Constants.Climber.climberEncoderPort));
    testing.add("Climber Extend Piston", new InstantCommand(
      () -> States.extendClimberPiston()
    ));
    testing.add("Climber Retract Piston", new InstantCommand(
      () -> States.retractClimberPiston()
    ));
    testing.add("Climber Extend Motor", new InstantCommand(
      () -> States.extendClimber()
    ));
    testing.add("Climber Retract Motor", new InstantCommand(
      () -> States.retractClimber()
    ));
  }

  public static boolean canClimb() {
    return States.climberState != States.ClimberStates.fullClimb;
  }

  public void extendClimber() {
    setClimberPosition(Constants.Climber.extendedCounts);
  }

  private void setClimberPosition(double climberPosition) {
    if(!canClimb()) return;
    if (climberPosition > Constants.Climber.climberHighLimit){
      climberPosition = Constants.Climber.climberHighLimit;
    }
    else if (climberPosition < Constants.Climber.climberLowLimit){
      climberPosition = Constants.Climber.climberLowLimit;
    }
    climberMotor.set(ControlType.kPosition, climberPosition);
  }

  public void setClimberPiston(boolean extended) {
    if(!canClimb()) return;
    if(extended) {
      climberPiston.set(Value.kForward);
    } else {
      climberPiston.set(Value.kReverse);
    }
  }

  public void setClimberPiston() {
    climberPiston.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(States.climberState) {
      case fullClimb:
      // climberPiston.set(Value.kForward);
      // climberMotor.set(ControlType.kDutyCycle, Constants.Climber.ClimberSpeed);
      // climberPiston.set(Value.kReverse);
      // climberPiston.set(Value.kForward);
      case extendClimber:
      setClimberPosition(Constants.Climber.extendedCounts);
      case retractClimber:
      setClimberPosition(Constants.Climber.retractedCounts);
      case extendClimberPiston:
      climberPiston.set(Value.kForward);
      case retractClimberPiston:
      climberPiston.set(Value.kReverse);
      case disabled:
      climberPiston.set(Value.kOff);
    }
    testing.add("Climber Encoder Value", positionEncoder.getCountsPerRevolution());
  }
}


// piston starts extended
      // bar climber goes up
      // piston retracts
      // piston extend
