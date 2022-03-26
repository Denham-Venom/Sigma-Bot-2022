// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazySparkMAX;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ClimberStates;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private LazySparkMAX climberMotor;
  private RelativeEncoder climbEncoder;
  private final ShuffleboardTab testing;
  private ClimberStates state = States.climberState;

  public Climber() {
    // Devices
    climberMotor = new LazySparkMAX(Constants.Climber.climberMotorConstants);
    // climberMotor.setStatusFrames(1000);
    climbEncoder = climberMotor.getEncoder();
    climbEncoder.setPosition(0);

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

  public void setPower(double power){
    if (power > 0 && climbEncoder.getPosition() < Constants.Climber.climberHighLimit){
      climberMotor.set(ControlType.kDutyCycle, power);
    }
    else if( power < 0 && climbEncoder.getPosition() > Constants.Climber.climberLowLimit){
      climberMotor.set(ControlType.kDutyCycle, power);
    }
    else{
      climberMotor.set(ControlType.kDutyCycle, 0);
    }
  }


  
  

  @Override
  public void periodic() {
    double climbEncoderPos = climbEncoder.getPosition();
    SmartDashboard.putNumber("Climber Encoder", climbEncoderPos);
  }
}


// piston starts extended
      // bar climber goes up
      // piston retracts
      // piston extend
