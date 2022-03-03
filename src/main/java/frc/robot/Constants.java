package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.Controllers.SparkConstants;
import frc.Controllers.TalonConstants;
import frc.lib.math.PIDGains;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(17.5);
        public static final double wheelBase = Units.inchesToMeters(17.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.86 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.1; //.6
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
        public static final boolean coastOnDisable = false;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = 335.47;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final double angleOffset = 27.509;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final double angleOffset = 254.79;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final double angleOffset = 205.40;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final TrajectoryConfig trajectoryConfig =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        public static final ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 
            0, 
            0, 
            AutoConstants.kThetaControllerConstraints
        );

        //whether "forward" for swerve is relative to front of robot (false) or to whole field (true)
        public static final boolean fieldRelative = true;
    }

    public static final class Shooter {
        public static final TalonConstants parentShooterConstants = 
            new TalonConstants(0, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None); //might need to change invert type
    
        public static final TalonConstants childShooterConstants = 
            new TalonConstants(0, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None); //might need to change invert type

        public static final TalonConstants hoodConstants = 
            new TalonConstants(0, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None); //might need to change invert type

        public static final TalonConstants turretConstants = 
            new TalonConstants(0, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None); //might need to change invert type

        public static final double shooterGearRatio = (1/1);
        public static final PIDGains shooterPID = new PIDGains(0.1, 0.0, 0.0, 0.046976); // might need to be changed

        public static final double hoodGearRatio = 0;
        public static final PIDGains hoodPID = new PIDGains(0, 0, 0, 0); //Definetly needs to be changed

        public static final double turretGearRatio = 0;
        public static final PIDGains turretPID = new PIDGains(0, 0, 0, 0);//Definetly needs to be changed

        //high angle = 70 degrees from straight, low angle = 10 degrees from straight up
        public static final double hoodHighLimit = 0;
        public static final double hoodLowLimit = 0;

        //high angle = 270; low angle = 0
        public static final double turretHighLimit = 0;
        public static final double turretLowLimit = 0;

        public static final boolean calibrationMode = false;

        /* Shooter Tuned Constants */
        public static final double[][] shooterMap = 
        // {distance, shooterRPM, shooterAngle}
        {
            {0, 0, 0}
        };

        public static final boolean autoAim = true;

    }

    public static final class Vision {
        public static final double goalHeight = Units.inchesToMeters(81.0 + (17.0 / 2.0));

        public static final double limelightHeight = Units.inchesToMeters(21.0);
        public static final Rotation2d limelightAngle = Rotation2d.fromDegrees(9.0);
    }

    public static final class Turret {
        public static final TalonConstants turretMotor1Constants = 
            new TalonConstants(0, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None); //might need to change invert type
    
        public static final TalonConstants turretMotor2Constants = 
            new TalonConstants(0, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None); //might need to change invert type
        public static final float hoodForwardLimit = 50; // might need to be changed
        public static final float hoodReverseLimit = 0;  // might need to be changed
        public static final double hoodEncoderOffset = 285; // might need to be changed
        public static final double turretForwardLimit = 50; // might need to be changed
        public static final double turretReverseLimit = 0; // might need to be changed
        public static final double turretEncoderOffset = 285; // might need to be changed
    }

    public static final class Climber {
        public static final SparkConstants climberMotorConstants = 
            new SparkConstants(0, MotorType.kBrushed, 35, IdleMode.kBrake, false); //might need to change invert type
    }

    public static final class Intake {
        public static final TalonConstants intakeMotorConstants = 
            new TalonConstants(0, talonCurrentLimit.supplyCurLim40, NeutralMode.Brake, InvertType.None); //might need to change invert type
        public static final SparkConstants spinUpMotorConstants = 
            new SparkConstants(0, MotorType.kBrushed, 35, IdleMode.kBrake, false); //might need to change invert type
        public static final double IntakeSpeed = 0;

        public static final int IntakeSolenoidForwardChannel = 0;
        public static final int IntakeSolenoidReverseChannel = 1;

    }

    public static final class talonCurrentLimit {
        public static final SupplyCurrentLimitConfiguration supplyCurLim40 = 
            new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
            
        public static final SupplyCurrentLimitConfiguration supplyCurLim30 = 
            new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);


        public static final Pose2d startPos = new Pose2d(7.606, 2.974, new Rotation2d(-1.894));
        //Starting Positions
            //Bottom
            //new Pose2d(7.606, 2.974, new Rotation2d(-1.894)),   //0 Defalt bottom starting position
            //new Pose2d(6.601, 2.546, new Rotation2d(-2.283)),   //1 Right bottom starting position
            //new Pose2d(8.439, 1.876, new Rotation2d(-1.561)),   //2 Left bottom starting position
            //new Pose2d(7.216, 1.99, new Rotation2d(-2.363)),    //3 Middle bottom starting postion
            //Top
            //new Pose2d(7.103, 4.871, new Rotation2d(2.742)),    //Defalt top starting position
            //new Pose2d(5.962, 3.958, new Rotation2d(3.141)),    //Right top starting position
            //new Pose2d(6.764, 5.712, new Rotation2d(2.035)),    //Left top starting position
            //new Pose2d(6.152, 5.156, new Rotation2d(2.399)),    //Middle top starting position
        public static final Pose2d[] rightPoints = 
        {
            new Pose2d(7.584, 1.084, new Rotation2d(-1.575)),
            new Pose2d(6.986, 1.269, new Rotation2d(2.25)),
            new Pose2d(5.572, 1.868, new Rotation2d(3.134)),
            new Pose2d(1.699, 1.811, new Rotation2d(-2.269)),
            new Pose2d(2.324, 2.111, new Rotation2d(0.583)),
            new Pose2d(4.771, 5.662, new Rotation2d(1.201)),
            new Pose2d(5.79, 6.247, new Rotation2d(2.353))
        };

        public static final Pose2d[] leftPoints = 
        {
            new Pose2d(5.381, 5.855, new Rotation2d(2.602)),
            new Pose2d(4.241, 4.15, new Rotation2d(-2.098)),
            new Pose2d(1.74, 1.74, new Rotation2d(-2.478)),
            new Pose2d(1.984, 1.868, new Rotation2d(0.429)),
            new Pose2d(4.485, 1.754, new Rotation2d(0.2)),
            new Pose2d(5.763, 1.983, new Rotation2d(-2.297)),
            new Pose2d(7.638, 1.112, new Rotation2d(-1.636))
        };
    }
}
