// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  private final JoystickButton togglePreshoot = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton switchGear = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton feedShooter = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton toggleIntakePiston = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton outtake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton toggleFieldRelative = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final POVButton extendClimber = new POVButton(driver, 0); //up
  private final POVButton retractClimber = new POVButton(driver, 180); //down
  private final POVButton extendClimberPiston = new POVButton(driver, 270); //left
  private final POVButton retractClimberPiston = new POVButton(driver, 90); //right

  /* Operator Buttons */
  private final JoystickButton opTogglePreshoot = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton opToggleUseIntakeSensors = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton opFeedShooter = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton opToggleIntakePiston = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton opSpins1 = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton opSpins2 = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton opAllowClimb = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton opDisallowClimb = new JoystickButton(operator, XboxController.Button.kBack.value);
  private final POVButton opExtendClimber = new POVButton(operator, 0); //up
  private final POVButton opRetractClimber = new POVButton(operator, 180); //down
  private final POVButton opExtendClimberPiston = new POVButton(operator, 270); //left
  private final POVButton opRetractClimberPiston = new POVButton(operator, 90); //right

  /* Subsystems */
  //private final PneumaticHub m_pHub;
  //private final PneumaticsControlModule m_pHub;
  private final Vision m_Vision;
  private final Shooter m_Shooter;
  private final Intaker m_Intaker;
  private final Swerve s_Swerve;
  private final Climber m_Climber;

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate subsystems
    //m_pHub = new PneumaticHub();
    //m_pHub = new PneumaticsControlModule();
    m_Vision = new Vision();
    m_Shooter = new Shooter(m_Vision);
    m_Intaker = new Intaker();
    s_Swerve = new Swerve(m_Vision);
    m_Climber = new Climber();

    
    s_Swerve.setDefaultCommand(new TeleopSwerve(
      s_Swerve, 
      driver, 
      translationAxis, 
      strafeAxis, 
      rotationAxis,  
      Constants.Swerve.openLoop
      ));
      
    configureShuffleboard();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  //Sendable Chooser for Autos
  // SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<AutoCommands.NumberOfBalls> m_chooseBall = new SendableChooser<>();
  SendableChooser<AutoCommands.StartingTarmac> m_chooseTarmac = new SendableChooser<>();
  SendableChooser<AutoCommands.StartingPosition> m_choosePosition = new SendableChooser<>();

  private void configureButtonBindings() {

    /* Driver Controller Buttons */
    // Driving
    zeroGyro.whenPressed(new InstantCommand(
      () -> s_Swerve.zeroGyro()
    ));
    switchGear.whenPressed(new InstantCommand(
      () -> s_Swerve.switchLowHighGear()
    ));
    toggleFieldRelative.whenPressed(new InstantCommand(
      () -> s_Swerve.toggleFieldRelative()
    ));

    
    // Intake
    intake.whileHeld(new StartEndCommand(
      () -> States.intake(), 
      () -> States.stopIntake()
    ));
    outtake.whileHeld(new StartEndCommand(
      () -> States.outtake(),
      () -> States.stopIntake()
    ));
    toggleIntakePiston.toggleWhenPressed(new InstantCommand(
      () -> States.toggleIntake()
    ));
    feedShooter.whileHeld(new StartEndCommand(
      () -> States.feed(),
      () -> States.stopIntake()
    ));

    // Shooter
    togglePreshoot.toggleWhenPressed(new StartEndCommand(
      () -> States.activateShooter(),
      () -> States.deactivateShooter()
    ));

    // Climber
    extendClimber.whileHeld(new StartEndCommand(
      () -> States.extendClimber(),
      () -> States.stopClimber()
    ));
    retractClimber.whileHeld(new StartEndCommand(
      () -> States.retractClimber(),
      () -> States.stopClimber()
    ));
    extendClimberPiston.whenPressed(new InstantCommand(
      () -> States.extendClimberPiston()
    ));
    retractClimberPiston.whenPressed(new InstantCommand(
      () -> States.retractClimberPiston()
    ));


    /* Operator Controller Buttons */
    // Swerve
    opSpins1.and(opSpins2).whenActive(new SequentialCommandGroup(
      new WaitCommand(1),
      new VictorySpins(s_Swerve)
    ));

    // Intake
    opFeedShooter.whileHeld(new StartEndCommand(
      () -> States.feed(), 
      () -> States.stopIntake()
    ));
    opToggleUseIntakeSensors.whenPressed(new InstantCommand(
      () -> m_Intaker.toggleUseSensors()
    ));
    opToggleIntakePiston.whenPressed(new InstantCommand(
      () -> States.toggleIntake()
    ));

    // Shooter
    opTogglePreshoot.toggleWhenPressed(new StartEndCommand(
      () -> States.activateShooter(),
      () -> States.deactivateShooter()
    ));

    // Climber
    opExtendClimber.whileHeld(new StartEndCommand(
      () -> States.extendClimber(),
      () -> States.stopClimber()
    ));
    opRetractClimber.whileHeld(new StartEndCommand(
      () -> States.retractClimber(),
      () -> States.stopClimber()
    ));
    opExtendClimberPiston.whenPressed(new InstantCommand(
      () -> States.extendClimberPiston()
    ));
    opRetractClimberPiston.whenPressed(new InstantCommand(
      () -> States.retractClimberPiston()
    ));
    opAllowClimb.whenPressed(new InstantCommand(
      () -> States.allowClimb()
    ));
    opDisallowClimb.whenPressed(new InstantCommand(
      () -> States.disallowClimb()
    ));

  };

  private void configureShuffleboard() {
    //Instatiate tabs
    Shuffleboard.getTab("Testing");
    Shuffleboard.getTab("Tuning");

    //Auto command chooser
    m_chooseBall.setDefaultOption("2 Balls", AutoCommands.NumberOfBalls.two);
    m_chooseBall.addOption("3 Balls", AutoCommands.NumberOfBalls.three);
    m_chooseBall.addOption("4 Balls", AutoCommands.NumberOfBalls.four);
    m_chooseBall.addOption("5 Balls", AutoCommands.NumberOfBalls.five);

    m_choosePosition.setDefaultOption("Middle of Tarmac", AutoCommands.StartingPosition.middle);
    m_choosePosition.addOption("Left of Tarmac", AutoCommands.StartingPosition.left);
    m_choosePosition.addOption("Right of Tarmac", AutoCommands.StartingPosition.right);

    m_chooseTarmac.setDefaultOption("Right Tarmac", AutoCommands.StartingTarmac.right);
    m_chooseTarmac.addOption("Left Tarmac", AutoCommands.StartingTarmac.left);
    // Puts the chooser on the dashboard
    //SmartDashboard.putData("auto", m_chooser);
    SmartDashboard.putData("Auto # Balls", m_chooseBall);
    SmartDashboard.putData("Auto Choose Position", m_choosePosition);
    SmartDashboard.putData("Auto Choose Tarmac", m_chooseTarmac);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    AutoCommands.StartingTarmac tarmac = m_chooseTarmac.getSelected();
    AutoCommands.setTarmac(tarmac);
    AutoCommands.StartingPosition start = m_choosePosition.getSelected();
    AutoCommands.setPosition(start);
    AutoCommands.NumberOfBalls balls = m_chooseBall.getSelected();
    AutoCommands.setBalls(balls);

    return AutoCommands.getSelectedAuto(s_Swerve);
  }
}
