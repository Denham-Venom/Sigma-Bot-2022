// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.States;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intaker;

public class ClimbControl extends CommandBase {

  private Climber m_Climber;
  private double power;

  public ClimbControl(Climber m_Climber, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Climber = m_Climber;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addRequirements(m_Climber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climber.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climber.setPower(0);
  }
}
