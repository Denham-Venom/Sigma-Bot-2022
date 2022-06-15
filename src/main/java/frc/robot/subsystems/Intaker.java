// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Color;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.Controllers.LazySparkMAX;
import frc.Controllers.LazyTalonFX;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.IntakeExtendStates;
import frc.robot.States.IntakeStates;

public class Intaker extends SubsystemBase {
  /** Creates a new Intaker. */
  private Shooter m_Shooter;
 
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
  private ColorSensorV3 colorSensor;
  private boolean useSensors = true;
  private boolean useColorSensor = true;
  private boolean useShooterTarget = true;
  private IntakeStates state = States.intakeState;
  private IntakeExtendStates pistonState = States.intakeExtendState;

  ShuffleboardTab Drivers = Shuffleboard.getTab("Drivers");
  NetworkTableEntry useIntakeSensors = Drivers.add("Use Sensors", false).getEntry();
  NetworkTableEntry useintakeColorSensor = Drivers.add("Use Color Sensor", false).getEntry();
  NetworkTableEntry intakeSensorValue = Drivers.add("intakeSensor", false).getEntry();
  NetworkTableEntry shooterSensorValue = Drivers.add("shooterSensor", false).getEntry();
  NetworkTableEntry intakeColorSensorValue = Drivers.add("intakeColorSensor", false).getEntry();
  NetworkTableEntry useShooterTargetEntry = Drivers.add("Checking Shooter RPM", true).getEntry();
  
  public Intaker(Shooter m_shooter){
    //Instantiate devices
    m_Shooter = m_shooter;
    indexerMotor = new LazyTalonFX(Constants.Intake.indexMotorConstants);
    indexerMotor.setStatusFrames(255);
    spinUpMotor = new LazySparkMAX(Constants.Intake.spinUpMotorConstants);
    spinUpMotor.setStatusFrames(1000);
    intakeMotor = new LazyTalonFX(Constants.Intake.intakeMotorConstants);
    intakeMotor.setStatusFrames(255);
    intakeExtend = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Intake.IntakeSolenoidForwardChannel, Constants.Intake.IntakeSolenoidReverseChannel);
    intakeSensor = new DigitalInput(6);
    shooterSensor = new DigitalInput(5);
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    //Configure shuffleboard
    testing = Shuffleboard.getTab("Testing");
    testing.add("Start Intake Motors", new InstantCommand(
      () -> States.intakeAndFeed()
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

  /**
   * Toggles the use of beam break sensors to frontload balls and auto retract intake
   */
  public void toggleUseSensors() {
    useSensors = !useSensors;
    useIntakeSensors.setBoolean(useSensors);
  }

  public void toggleUseColorSensors() {
    useColorSensor = !useColorSensor;
    useintakeColorSensor.setBoolean(useColorSensor);
  }

  public void toggleCheckShooter() {
    useShooterTarget = !useShooterTarget;
    useShooterTargetEntry.setBoolean(useShooterTarget);
  }


  @Override
  public void periodic() {

    intakeSensorValue.setBoolean(!intakeSensor.get());
    shooterSensorValue.setBoolean(!shooterSensor.get());
    
    // Retract intake when full
    if(useSensors && !intakeSensor.get() && !shooterSensor.get() && States.intakeState != States.IntakeStates.outtaking && States.intakeState != States.IntakeStates.intakeAndFeed){
      States.retractIntake();
    }

    // Check for state update or if intaking
    if(States.intakeState != state || state == States.IntakeStates.intaking || state == States.IntakeStates.feeding) {
      state = States.intakeState;
      switch(States.intakeState) {
        case intaking:
          if(useSensors) {
            if(!intakeSensor.get()) { //ball intake
              if(!shooterSensor.get()) { //ball shooter
                spinUpMotor.set(ControlType.kDutyCycle, 0);
                indexerMotor.set(ControlMode.PercentOutput, 0);
              } else { //no ball shooter
                indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
                spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
                intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
              }
            } else if(!shooterSensor.get()) { //no ball intake, ball shooter
              indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
              spinUpMotor.set(ControlType.kDutyCycle, 0);
              intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
            } else { //no balls
              indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
              spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
              intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
            }
          } else {
            spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
            indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
            intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
          }

          if(useColorSensor) {


            if(Constants.alliance == Alliance.Blue) {
              if(colorSensor.getBlue() >= Constants.blueBallBlue) {
                new ParallelDeadlineGroup(
                  new WaitCommand(0.8),
                  new InstantCommand(() -> States.intake()));
              }
              else if (colorSensor.getRed() > Constants.redBallRed) {
                new ParallelDeadlineGroup(
                  new WaitCommand(0.8),
                  new InstantCommand(() -> States.outtake()));
              }
            }
            if(Constants.alliance == Alliance.Red) {
              if(colorSensor.getRed() == Constants.redBallRed) {
                new ParallelDeadlineGroup(
                  new WaitCommand(0.8),
                  new InstantCommand(() -> States.intake()));
              }
              else if (colorSensor.getBlue() == Constants.blueBallBlue) {
                new ParallelDeadlineGroup(
                  new WaitCommand(0.8),
                  new InstantCommand(() -> States.outtake()));
              }
            }
          }
          break;

        case outtaking:
        // reverses the intake motor
          intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.intakeSpeed);
          States.deployIntake();
          break;

        case feeding: 
        // Runs all intake/indexer motors
          //TODO only spin if drivetrain and hood are ready
          if(!useShooterTarget) {
            spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new WaitCommand(0.15),
              new InstantCommand(() -> indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed))
            ));
          } else if(m_Shooter.isShooterReady()){
            spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
            indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed);
          } else {
            spinUpMotor.set(ControlType.kDutyCycle, 0);
            indexerMotor.set(ControlMode.PercentOutput, 0);
          }
          //intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
          break;

        case reverseFeeding:
          indexerMotor.set(ControlMode.PercentOutput, -Constants.Intake.indexSpeed);
          spinUpMotor.set(ControlType.kDutyCycle, -Constants.Intake.spinupSpeed);
          intakeMotor.set(ControlMode.PercentOutput, -Constants.Intake.intakeSpeed);
          break;

        case disabled:
        // Stops all intake/indexer motors
          indexerMotor.set(ControlMode.PercentOutput, 0);
          spinUpMotor.set(ControlType.kDutyCycle, 0);
          intakeMotor.set(ControlMode.PercentOutput, 0);
          break;

        case intakeAndFeed: //TODO maybe add auto intake deploy here
          States.deployIntake();
          spinUpMotor.set(ControlType.kDutyCycle, Constants.Intake.spinupSpeed);
          CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
            new WaitCommand(0.5),
            new InstantCommand(() -> indexerMotor.set(ControlMode.PercentOutput, Constants.Intake.indexSpeed))
          ));
          intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.intakeSpeed);
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

