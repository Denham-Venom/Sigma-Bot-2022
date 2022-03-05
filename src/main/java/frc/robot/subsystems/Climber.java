// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  private DutyCycleEncoder positionEncoder;
  private final ShuffleboardTab testing;
  public Climber(PneumaticHub m_pHub) {
    testing = Shuffleboard.getTab("Testing");
    climberMotor = new LazySparkMAX(Constants.Climber.climberMotorConstants);
    climberPiston = m_pHub.makeDoubleSolenoid(Constants.Climber.ClimberSolenoidForwardChannel, Constants.Climber.ClimberSolenoidReverseChannel);
    positionEncoder = new DutyCycleEncoder(new DigitalInput(Constants.Climber.climberEncoderPort));
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

  public void setClimberPosition(double climberPosition) {
    if (climberPosition > Constants.Climber.climberHighLimit){
      climberPosition = Constants.Climber.climberHighLimit;
    }
    else if (climberPosition < Constants.Climber.climberLowLimit){
      climberPosition = Constants.Climber.climberLowLimit;
    }
    climberMotor.set(ControlType.kPosition, climberPosition);
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
      setClimberPosition(0);
      case retractClimber:
      setClimberPosition(0);
      case extendClimberPiston:
      climberPiston.set(Value.kForward);
      case retractClimberPiston:
      climberPiston.set(Value.kReverse);
      case disabled:
      climberPiston.set(Value.kOff);
    }
    SmartDashboard.putNumber("Climber Encoder Value", positionEncoder.get());
  }
}


// piston starts extended
      // bar climber goes up
      // piston retracts
      // piston extend
