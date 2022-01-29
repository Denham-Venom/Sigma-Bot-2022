// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazyTalonFX;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.IntakeStates;

public class Intaker extends SubsystemBase {
  /** Creates a new Intaker. */
  
  // indexer 1
  private LazyTalonFX intakeMotor1;
  // indexer 2
  private LazyTalonFX intakeMotor2;
  // intaker
  private LazyTalonFX intakeMotor3;
  private DigitalInput intakeSensor;
  private DigitalInput shooterSensor;
  public Intaker() {
    intakeMotor1 = new LazyTalonFX(Constants.Intake.intakeMotorConstants);
    intakeMotor2 = new LazyTalonFX(Constants.Intake.intakeMotorConstants);
    intakeMotor3 = new LazyTalonFX(Constants.Intake.intakeMotorConstants);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(States.intakeState) {
      case intaking:
        if(!intakeSensor.get()) {
          intakeMotor1.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
          intakeMotor2.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
          intakeMotor3.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        }
        else if(!shooterSensor.get()) {
          intakeMotor1.set(ControlMode.PercentOutput, 0);
          intakeMotor2.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
          intakeMotor3.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        }
        else {
          intakeMotor1.set(ControlMode.PercentOutput, 0);
          intakeMotor2.set(ControlMode.PercentOutput, 0);
          intakeMotor3.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        }
      case feeding:
        intakeMotor1.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        intakeMotor2.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
        intakeMotor3.set(ControlMode.PercentOutput, Constants.Intake.IntakeSpeed);
      case disabled:
        intakeMotor1.set(ControlMode.PercentOutput, 0);
        intakeMotor2.set(ControlMode.PercentOutput, 0);
        intakeMotor3.set(ControlMode.PercentOutput, 0);
    }
  }
}
