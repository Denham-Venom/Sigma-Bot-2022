// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Bottom4BallLeft extends SequentialCommandGroup {
  /** Creates a new Bottom4BallLeft. */
  public Bottom4BallLeft(Swerve s_Swerve, Shooter m_Shooter) {
    
    Trajectory bottom4BallLeftPart1 = TrajectoryGenerator.generateTrajectory(
    List.of(),
    Constants.Swerve.trajectoryConfig);

    Trajectory bottom4BallLeftPart2 = TrajectoryGenerator.generateTrajectory(
    List.of(),
    Constants.Swerve.trajectoryConfig);

    Trajectory bottom4BallLeftPart3 = TrajectoryGenerator.generateTrajectory(
    List.of(),
    Constants.Swerve.trajectoryConfig);



    addCommands();
  }
}
