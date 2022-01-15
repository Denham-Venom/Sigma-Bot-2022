// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Controllers.LazyTalonFX;
import frc.robot.Constants;

public class Turreter extends SubsystemBase {
  /** Creates a new Turret. */
  public LazyTalonFX hoodMotor;
  public LazyTalonFX turretMotor;
  public Turreter() {
    hoodMotor = new LazyTalonFX(Constants.Turret.turretMotor1Constants);
    turretMotor = new LazyTalonFX(Constants.Turret.turretMotor2Constants);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
