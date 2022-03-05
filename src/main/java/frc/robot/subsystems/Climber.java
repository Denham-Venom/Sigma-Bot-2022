// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazySparkMAX;
import frc.Controllers.LazyTalonFX;
import frc.robot.Constants;
import frc.robot.States;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private LazySparkMAX climberMotor;
  private DoubleSolenoid shooterPiston;
  public Climber(PneumaticHub m_pHub) {
    climberMotor = new LazySparkMAX(Constants.Climber.climberMotorConstants);
    shooterPiston = m_pHub.makeDoubleSolenoid(Constants.Climber.ClimberSolenoidForwardChannel, Constants.Climber.ClimberSolenoidReverseChannel);
    SmartDashboard.putData("Climber Motor", new StartEndCommand(
      () -> States.extendClimber(),
      () -> States.retractClimber()
    ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(States.climberState) {
      case fullClimb:
      case extendClimber:
      case retractClimber:
      case disabled:
        
    }
  }
}
