// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intaker;

public class Intake extends CommandBase {

  private Intaker m_Intaker;
  private double power;
  public Intake(double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.power = power;
    addRequirements(m_Intaker);
  }

  public Intake(Intaker m_Intaker, double power) {
    this.m_Intaker = m_Intaker;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intaker.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intaker.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
