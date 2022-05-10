package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public TalonFXConfiguration shooterFXConfig;
    public TalonSRXConfiguration shooterSRXConfig;

    public TalonFXConfiguration indexerFXConfig;
    public TalonFXConfiguration intakeFXConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        shooterFXConfig = new TalonFXConfiguration();
        shooterSRXConfig = new TalonSRXConfiguration();

        indexerFXConfig = new TalonFXConfiguration();
        intakeFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
        swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = Constants.talonCurrentLimit.supplyCurLim30;

        /* Swerve Drive Motor Configuration */
        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = Constants.talonCurrentLimit.supplyCurLim40;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Shooter Motor Configuration */
        shooterFXConfig.slot0.kP = Constants.Shooter.shootKP;
        shooterFXConfig.slot0.kI = Constants.Shooter.shootKI;
        shooterFXConfig.slot0.kD = Constants.Shooter.shootKD;
        shooterFXConfig.slot0.kF = Constants.Shooter.shootKF;
        shooterFXConfig.supplyCurrLimit = Constants.talonCurrentLimit.supplyCurLim40;

        /* Hood Motor Configuration */
        shooterSRXConfig.continuousCurrentLimit = (int) Constants.talonCurrentLimit.supplyCurLim30.currentLimit;
        shooterSRXConfig.peakCurrentLimit = (int) Constants.talonCurrentLimit.supplyCurLim30.triggerThresholdCurrent;
        shooterSRXConfig.peakCurrentDuration = (int) (Constants.talonCurrentLimit.supplyCurLim30.triggerThresholdTime * 1000);

        /* Indexer Motor Configuration */
        indexerFXConfig.supplyCurrLimit = Constants.talonCurrentLimit.supplyCurLim40;

        /* Intake Motor Configuration */
        intakeFXConfig.supplyCurrLimit = Constants.talonCurrentLimit.supplyCurLim40;





        

    }

}