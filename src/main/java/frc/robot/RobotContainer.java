// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.States.IntakeExtendStates;
import frc.robot.States.IntakeStates;
import frc.robot.States.ShooterStates;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton highLowGearButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton switchShooterState = new JoystickButton(driver,XboxController.Button.kX.value);
  private final JoystickButton testHood = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton outakeButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final POVButton intakeExtendButton = new POVButton(driver, 180);
  private final POVButton intakeRetractButton = new POVButton(driver, 0);
  private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kStart.value);

  /* Operator Buttons */
  private final JoystickButton operatorShootButton = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton shooterActivateButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton shooterDeactivateButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton operatorIntakeButton = new JoystickButton(operator, XboxController.Button.kY.value);
  private final POVButton operatorIntakeExtendButton = new POVButton(operator, 180);
  private final POVButton operatorIntakeRetractButton = new POVButton(operator, 0);
  private final JoystickButton operatorFeedButton = new JoystickButton(operator, XboxController.Button.kX.value);

  /* Subsystems */
  private final Intaker m_Intaker = new Intaker(new PneumaticHub());
  private final Vision m_Vision = new Vision();
  private final Swerve s_Swerve = new Swerve(m_Vision);
  private final Shooter m_Shooter = new Shooter(m_Vision);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = Constants.Swerve.fieldRelative;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
      //Sendable Chooser for Autos
      SendableChooser<Command> m_chooser = new SendableChooser<>();

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    highLowGearButton.whenPressed(new InstantCommand(() -> s_Swerve.switchLowHighGear()));

      // Intake
      intakeButton.whileHeld(new StartEndCommand(
        () -> States.intake(), 
        () -> States.stopIntake()
      ));
      outakeButton.whileHeld(new Intake(m_Intaker));

      // Shooter
      operatorShootButton.toggleWhenPressed(new StartEndCommand(
        () -> States.activateShooter(),
        () -> States.deactivateShooter()
      ));
      operatorFeedButton.whileHeld(new StartEndCommand(
        () -> States.feed(), 
        () -> States.stopIntake()
      ));

      // Test Hood
      testHood.whenPressed(new InstantCommand(() -> m_Shooter.setHoodAngle(0)));
    
      /* Operator Buttons */
      operatorIntakeButton.whileHeld( new StartEndCommand(
        () -> States.intake(),
        () -> States.stopIntake()
      ));

      //Auto command chooser
      m_chooser.setDefaultOption("Right5Ball", new Right5Ball(s_Swerve));
      m_chooser.addOption("Right4Ball", new Right4Ball(s_Swerve));
      m_chooser.addOption("Right3Ball", new Right4Ball(s_Swerve));
      m_chooser.addOption("Right2Ball", new Right4Ball(s_Swerve));
      m_chooser.addOption("Left5Ball", new Right4Ball(s_Swerve));
      m_chooser.addOption("Left4Ball", new Right4Ball(s_Swerve));
      m_chooser.addOption("Left3Ball", new Right4Ball(s_Swerve));
      m_chooser.addOption("Left2Ball", new Right4Ball(s_Swerve));
      // Puts the chooser on the dashboard
      SmartDashboard.putData("auto", m_chooser);
  };

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
