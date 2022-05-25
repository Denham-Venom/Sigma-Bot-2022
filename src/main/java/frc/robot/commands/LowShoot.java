// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.States;

public class LowShoot extends CommandBase {

  private Timer t;
  private double waitDur = 0.1; // in seconds

  /** Creates a new LowShoot. */
  public LowShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    t = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // resets timer back to 0 and starts it
    t.reset();
    t.start();
    // sets the shooter state to the low preshoot state
    States.setActiveShooterMode(States.ShooterStates.lowPreShoot);
    // runs shooter
    States.activateShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Once the timer has lasted 0.1 seconds, stops and resets timer.
    // Then feeds the cargo
    if(t.get() > waitDur) {
      t.stop();
      t.reset();
      States.feed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops intake and shooter, then sets the shooter state to preshoot
    States.stopIntake();
    States.deactivateShooter();
    States.setActiveShooterMode(States.ShooterStates.preShoot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
