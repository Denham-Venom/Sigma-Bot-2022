package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.Controllers.SparkConstants;
import frc.lib.Controllers.TalonFxConstants;
import frc.lib.Controllers.TalonSRXConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    /* General constants */
    public static final boolean tuningMode = false;
    public static final double stickDeadband = 0.1;
    public static final String driverTab = "Drivers";
    public static final int ballCamFPS = 12;

    /* Container for swerve subsystem constants */
    public static final class Swerve {
        /* Behavior constants */
        public static final boolean openLoop = true;

        /* Device IDs */
        public static final int pigeonID = 1;

        /* Drivetrain Mechanism Properties */
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        public static final double trackWidth = Units.inchesToMeters(19.5);
        public static final double wheelBase = Units.inchesToMeters(19.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.90); //changed from 3.9 for new neoprene tread
        public static final double wheelCircumference = wheelDiameter * Math.PI; //0.3110484 m
        public static final double driveGearRatio = (6.75 / 1.0); //6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1
        public static final SwerveDriveKinematics swerveKinematics = 
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            );

        /* Ramping values */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.2;/*0.1;*/ /*.6*/
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0; //12
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (1.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Swerve Gear Scaling Values */
        public static final double highGearValue = 0.7;
        public static final double lowGearValue = 0.3;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
        public static final boolean coastOnDisable = true;

        /* Motor Inverts */
        public static final InvertType driveMotorInvert = InvertType.None;
        public static final InvertType angleMotorInvert = InvertType.None;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final double angleOffset = 228.515625 - 180; //This might be the actual one 48.33984375;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 2;
            public static final double angleOffset = 73.7402344; //This one might be right 73.212890625;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 3;
            public static final double angleOffset = 220.341796875 - 180 ; //This one might be correct 38.144;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 4;
            public static final double angleOffset = 297.509765625; //This one might be correct 297.421875;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Rotational PID for aiming while shooting */        
        public static final double thetaKP = -5.0;
        public static final double thetaKI = 0.0;
        public static final double thetaKD = -0.1;
        public static double thetaTolerance = 0.07;
    }

    /* Container for shooter subsystem constants */
    public static final class Shooter {
        /* Shooter behavior constants */
        public static final boolean calibrationMode = false;

        /* IDs for motors and sensors */
        private static final int parentShooterID = 9;
        private static final int childShooterID = 10;

        /* Motor constants */
        public static final TalonFxConstants parentShooterConstants = new TalonFxConstants(
            parentShooterID, 
            Robot.ctreConfigs.shooterFXConfig,
            NeutralMode.Coast, 
            InvertType.InvertMotorOutput,
            false
        );

        public static final TalonFxConstants childShooterConstants = new TalonFxConstants(
            childShooterID, 
            Robot.ctreConfigs.shooterFXConfig,
            NeutralMode.Coast, 
            InvertType.OpposeMaster,
            true
        );
        
        /* Mechanism properties */
        public static final double shooterGearRatio = (1/1);
        public static final double tolerance = 100;

        /* Shooter characterization and tuning values */
        public static final double shootKs = 0.75347;
        public static final double shootKv = 0.11309;
        public static final double shootKa = 0.0073;
        public static final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(shootKs, shootKv, shootKa);

        public static final double shootKP = 0.15;
        public static final double shootKI = 0.0;
        public static final double shootKD = 1.0;
        public static final double shootKF = 0.05258; //1023.0 / Conversions.RPMToFalcon(5700, shooterGearRatio); 

        /* Shooter Calibration Values */
        public static final double[][] shooterMap = 
        {// {distance (m), shooter speed (RPM), shooter angle (degrees from horiz)}
            {1.27 + Units.feetToMeters(4)/2, 2000, 15},
            {1.75 + Units.feetToMeters(4)/2, 2000, 17},
            {2.17 + Units.feetToMeters(4)/2, 2050, 20},
            {2.67 + Units.feetToMeters(4)/2, 2100, 21},
            {2.85 + Units.feetToMeters(4)/2, 2175, 23.3},
            {2.93 + Units.feetToMeters(4)/2, 2175, 22},
            {3.19 + Units.feetToMeters(4)/2, 2250, 23},
            {3.39 + Units.feetToMeters(4)/2, 2300, 23.9},
            {3.67 + Units.feetToMeters(4)/2, 2400, 23},
            {3.91 + Units.feetToMeters(4)/2, 2300, 24.5},
            {4.08 + Units.feetToMeters(4)/2, 2350, 25.7},
            {4.18 + Units.feetToMeters(4)/2, 2425, 25},
            {4.74 + Units.feetToMeters(4)/2, 2550, 27}, 
            {5.66 + Units.feetToMeters(4)/2, 2650, 29}
        };

        public static final double[] shooterLowMap = 
        // distance, shooter speed, shooter angle
        {1.83, 900, 14.00};
    }

    public static final class Hood {
        private static final int hoodID = 12;
        public static final TalonSRXConstants hoodConstants = new TalonSRXConstants(
            hoodID, 
            Robot.ctreConfigs.shooterSRXConfig,
            NeutralMode.Brake, 
            InvertType.InvertMotorOutput,
            true
        );
        
        public static final int hoodEncoderAbsoluteChannel = 0;
        public static final int[] hoodEncoderRelativeChannels = {1, 2};
        public static final boolean hoodEncoderInverted = true;
        public static final int hoodLimitSwitchID = 4;

        public static final int hoodEncoderCountsPerRev = 8192;
        public static final double hoodLowLimit = 10;
        public static final double hoodHighLimit = 90;

        public static final double hoodGearRatio = 18.0/42.0;
        public static final double hoodAngleOffset = 10.0;        
        
        public static final double hoodKP = 0.05;
        public static final double hoodKI = 0.0;
        public static final double hoodKD = 0.001;
        public static final double hoodKF = 0.12;

        public static final double hoodDownFF = -0.05;
        public static final double hoodControllerToleranceDegrees = 0.3;
    }

    /* Container for climber subsystem constants */
    public static final class Climber {
        /* Motor and device IDs */
        private static final int climberMotorID = 15;
        public static final boolean climberEncoderInverted = false;

        /* Motor constants */
        public static final SparkConstants climberMotorConstants = new SparkConstants(
            climberMotorID, 
            MotorType.kBrushless, 
            sparkCurrentLimit.sparkCurLimit40, 
            IdleMode.kBrake, 
            true //invert
        );
        
        /* Tuned values */
        public static final double ClimberSpeed = 0.5;

        /* Mechanism properties */
        public static final double climberHighLimit = 200.2992;
        public static final double climberLowLimit = 0;
        public static final double climberGearRatio = 12.0;
    }

    /* Container for intake subsystem constants */
    public static final class Intake {
        /* Motor and device IDs */
        private static final int intakeID = 11;
        private static final int indexID = 13;
        private static final int spinupID = 14;
        public static final int IntakeSolenoidForwardChannel = 0;
        public static final int IntakeSolenoidReverseChannel = 1;

        /* Motor constants */
        public static final TalonFxConstants intakeMotorConstants = new TalonFxConstants(
            intakeID, 
            Robot.ctreConfigs.intakeFXConfig,
            NeutralMode.Coast, 
            InvertType.InvertMotorOutput,
            true
        );
        public static final TalonFxConstants indexMotorConstants = new TalonFxConstants(
            indexID, 
            Robot.ctreConfigs.indexerFXConfig,
            NeutralMode.Brake, 
            InvertType.None,
            true
        );
        public static final SparkConstants spinUpMotorConstants = new SparkConstants(
            spinupID, 
            MotorType.kBrushless, 
            sparkCurrentLimit.sparkCurLimit40, 
            IdleMode.kBrake, 
            false
        );
        
        /* Tuned speed constants */
        public static final double intakeSpeed = 0.5;
        public static final double indexSpeed = 0.6;
        public static final double spinupSpeed = 0.5;
    }    

    /* Container for vision subsystem constants */
    public static final class Vision {        
        public static final Translation2d robotToShooterOffset = new Translation2d(Units.inchesToMeters(-3.12), 0);
        public static final Translation2d shooterToLLOffset = new Translation2d(Units.inchesToMeters(-8.45), 0);
        public static final Rotation2d shooterAngle = Rotation2d.fromDegrees(180);

        public static final double practiceFieldHeight = Units.inchesToMeters(103);
        public static final double compFieldHeight = Units.inchesToMeters(104);

        public static final double goalDiameter = Units.inchesToMeters(53.375);
        public static final double goalHeight = compFieldHeight;
        public static final Pose2d goalPose = new Pose2d(Units.inchesToMeters(324.0), Units.inchesToMeters(162.0), new Rotation2d());

        public static final double limelightHeight = Units.inchesToMeters(21.0);//TODO: Check
        public static final Rotation2d limelightAngle = Rotation2d.fromDegrees(47);//TODO: Check
    }

    public static final class PoseEstimator {
        public static final Matrix<N3,N1> stateStdDevs = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(0.01)); 
        public static final Matrix<N1,N1> gyroStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01));
        public static final Matrix<N3,N1> visionStdDevs = VecBuilder.fill(0.045, 0.045, Units.degreesToRadians(0.01));
    }

    /* Current limiting configurations */
    public static final class talonCurrentLimit {
        public static final SupplyCurrentLimitConfiguration supplyCurLim40 = 
            new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
            
        public static final SupplyCurrentLimitConfiguration supplyCurLim30 = 
            new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
    }

    public static final class sparkCurrentLimit {
        public static final int sparkCurLimit40 = 300;
    }

    /* Container for constants for autonomous routines */
    public static final class AutoConstants {
        /* Auto Swerve Controller Constants */
        public static final double kPXController = 1.0;
        public static final double kPYController = 1.0;
        public static final double kPThetaController = 5.0;

        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    }    
}
