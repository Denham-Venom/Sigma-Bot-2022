// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.States;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.Intaker;

public class Intake extends CommandBase {

  private Intaker m_Intaker;
  public Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intaker);
  }

  public Intake(Intaker m_Intaker) {
    this.m_Intaker = m_Intaker;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    States.intakeState = IntakeStates.intaking;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    States.intakeState = IntakeStates.disabled;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
