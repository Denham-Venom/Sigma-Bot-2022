package frc.lib.SwerveController;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveControllerConfig {
    public Subsystem requirements;
    public Supplier<Pose2d> m_pose;
    public SwerveDriveKinematics m_kinematics;
    public Consumer<SwerveModuleState[]> m_outputModuleStates;
    public double maxVelocity;
    public double maxAcceleration;
    public double kPXController;
    public double kPYController;
    public double kPThetaController;    
    
    public SwerveControllerConfig(Subsystem requirements, Supplier<Pose2d> m_pose, Consumer<SwerveModuleState[]> m_outputModuleStates, SwerveDriveKinematics m_kinematics, double maxVelocity, double maxAcceleration, double kPXController, double kPYController, double kPThetaController){
        this.requirements = requirements;
        this.m_pose = m_pose;
        this.m_outputModuleStates = m_outputModuleStates;
        this.m_kinematics = m_kinematics;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.kPXController = kPXController;
        this.kPYController = kPYController;
        this.kPThetaController = kPThetaController;
    }
}
