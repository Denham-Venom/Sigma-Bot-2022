// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazySparkMAX;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ClimberStates;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private LazySparkMAX climberMotor;
  private DoubleSolenoid climberPiston;
  private final ShuffleboardTab testing;
  private ClimberStates state = States.climberState;

  public Climber(PneumaticHub m_pHub) {
    // Devices
    climberMotor = new LazySparkMAX(Constants.Climber.climberMotorConstants);
    climberPiston = m_pHub.makeDoubleSolenoid(Constants.Climber.ClimberSolenoidForwardChannel, Constants.Climber.ClimberSolenoidReverseChannel);
      
    // Shuffleboard
    testing = Shuffleboard.getTab("Testing");
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
  
  /**
   * Checks if the climber is in a climbing state
   * @return
   */
  public static boolean canClimb() {
    return States.climberState != States.ClimberStates.disabled;
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
    //if(!canClimb()) return;
    if(extended) {
      climberPiston.set(Value.kForward);
    } else {
      climberPiston.set(Value.kReverse);
    }
  }

  private void setClimberMotor(double demand) {
    climberMotor.set(demand);
  }

  public void setClimberPiston() {
    climberPiston.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(States.climberState != state) {
      state = States.climberState;
      switch(States.climberState) {
        case extendClimber:
        //setClimberMotorSafe(Constants.Climber.ClimberSpeed);
        setClimberMotor(Constants.Climber.ClimberSpeed);
        break;
        case retractClimber:
        //setClimberMotorSafe(-Constants.Climber.ClimberSpeed);
        setClimberMotor(-Constants.Climber.ClimberSpeed);
        break;
        case extendClimberPiston:
        climberPiston.set(Value.kForward);
        break;
        case retractClimberPiston:
        climberPiston.set(Value.kReverse);
        break;
        case disabled:
        climberPiston.set(Value.kOff);
        setClimberMotor(0);
        break;
      }
    }
  }
}


// piston starts extended
      // bar climber goes up
      // piston retracts
      // piston extend
