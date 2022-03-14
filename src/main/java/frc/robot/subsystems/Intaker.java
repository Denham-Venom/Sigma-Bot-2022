// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final ShuffleboardTab testing;

  private DoubleSolenoid intakeExtend;
  private DigitalInput intakeSensor;
  private DigitalInput shooterSensor;
  private boolean useSensors = false;
  private IntakeStates state = States.intakeState;
  private IntakeExtendStates pistonState = States.intakeExtendState;
  public Intaker(PneumaticHub m_pHub){//, Consumer<RelativeEncoder> hoodEncoderGetter) {
    testing = Shuffleboard.getTab("Testing");

    indexerMotor = new LazyTalonFX(Constants.Intake.indexMotorConstants);
    spinUpMotor = new LazySparkMAX(Constants.Intake.spinUpMotorConstants);
    intakeMotor = new LazyTalonFX(Constants.Intake.intakeMotorConstants);
    intakeExtend = m_pHub.makeDoubleSolenoid(Constants.Intake.IntakeSolenoidForwardChannel, Constants.Intake.IntakeSolenoidReverseChannel);
    //hoodEncoderGetter.accept(spinUpMotor.getAlternateEncoder(Type.kQuadrature, Constants.Shooter.hoodEncoderCountsPerRev));
    testing.add("Start Intake Motors", new InstantCommand(
      () -> States.feed()
    ));
    testing.add("Stop Intake Motors", new InstantCommand(
      () -> States.stopIntake()
    ));

    testing.add("Extend Intaker", new InstantCommand(
      () -> States.deployIntake()
    ));
    testing.add("Retract Intaker", new InstantCommand(
      () -> States.retractIntake()
    ));


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(States.intakeState != state) {
      state = States.intakeState;
      switch(States.intakeState) {
        case intaking:
          if(useSensors) {
            if(!intakeSensor.get()) {
              if(!shooterSensor.get()) {
                spinUpMotor.set(ControlType.kDutyCycle, 0);
              } else {
                spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
              }
              indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
              intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
            }
            else if(!shooterSensor.get()) {
              indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
              spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
              intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
            } else {
              indexerMotor.set(ControlMode.PercentOutput, 0);
              spinUpMotor.set(ControlType.kDutyCycle, 0);
              intakeMotor.set(ControlMode.PercentOutput, 0);
            }
          } else {
            spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
            indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
            intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
          }
          break;
        case outtaking:
        // reverses the intake motor
          intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.intakeSpeed);
          break;
        case feeding: 
        // Runs all intake/indexer motors
          indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
          spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
          intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
          break;
        case disabled:
        // Stops all intake/indexer motors
          indexerMotor.set(ControlMode.PercentOutput, 0);
          spinUpMotor.set(ControlType.kDutyCycle, 0);
          intakeMotor.set(ControlMode.PercentOutput, 0);
          break;
      }
    }

    if(States.intakeExtendState != pistonState) {
      pistonState = States.intakeExtendState;
      switch(States.intakeExtendState) {
        case deployIntake:
          intakeExtend.set(Value.kForward);
          break;
        case retractIntake:
          intakeExtend.set(Value.kReverse); 
          break;
        case disabled:
          intakeExtend.set(Value.kOff);
          break;
        }
      }
    }
  }

