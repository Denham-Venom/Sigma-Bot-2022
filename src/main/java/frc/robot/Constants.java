package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.Controllers.SparkConstants;
import frc.Controllers.TalonConstants;
import frc.lib.math.Conversions;
import frc.lib.math.PIDGains;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.SwerveTrajectoryWaypoint;

public final class Constants {

    /* General constants */
    public static final boolean tuningMode = false;
    public static final double stickDeadband = 0.1;
    public static final String driverTab = "Drivers";


    /* Container for swerve subsystem constants */
    public static final class Swerve {

        /* Behavior constants */
        //whether "forward" for swerve is relative to front of robot (false) or to whole field (true)
        //public static final boolean fieldRelative = true;
        public static final boolean openLoop = true;

        /* Device IDs */
        public static final int pigeonID = 1;

        /* Drivetrain Mechanism Properties */
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        public static final double trackWidth = Units.inchesToMeters(17.5);
        public static final double wheelBase = Units.inchesToMeters(17.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.90);
        public static final double wheelCircumference = wheelDiameter * Math.PI; //0.3110484 m
        public static final double driveGearRatio = (6.86 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Ramping values */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0; 

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
        public static final double highGearValue = 1.0;
        public static final double lowGearValue = 0.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
        public static final boolean coastOnDisable = true;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final double angleOffset = 48.33984375;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 2;
            public static final double angleOffset = 73.212890625;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 3;
            public static final double angleOffset = 38.144;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 4;
            public static final double angleOffset = 297.421875;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Translation PID */
        public static final double xKP = 1;
        public static final double xKI = 0;
        public static final double xKD = 0;
        public static final double yKP = 1;
        public static final double yKI = 0;
        public static final double yKD = 0;

        /* Rotational PID */
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, 
            kMaxAngularSpeedRadiansPerSecondSquared
        );
        public static final double thetaKP = -5.0;
        public static final double thetaKI = 0.0;
        public static final double thetaKD = -0.1;
        public static final double transRateLimit = Double.MAX_VALUE;
        public static final double rotRateLimit = Double.MAX_VALUE;
        public static double thetaTolerance = 0.07;

    }



    /* Container for shooter subsystem constants */
    public static final class Shooter {

        /* Shooter behavior constants */
        public static final boolean calibrationMode = false;
        public static final boolean autoAim = false; //TODO set to true


        /* IDs for motors and sensors */
        private static final int parentShooterID = 9;
        private static final int childShooterID = 10;
        private static final int hoodID = 12;
        public static final int hoodEncoderAbsoluteChannel = 0;
        public static final int[] hoodEncoderRelativeChannels = {1, 2};
        public static final boolean hoodEncoderInverted = true;
        public static final int hoodLimitSwitchID = 4;

        /* Motor constants */
        public static final TalonConstants parentShooterConstants = new TalonConstants(
            parentShooterID, 
            talonCurrentLimit.supplyCurLim40, 
            NeutralMode.Coast, 
            InvertType.InvertMotorOutput
        );
        public static final TalonConstants childShooterConstants = new TalonConstants(
            childShooterID, 
            talonCurrentLimit.supplyCurLim40, 
            NeutralMode.Coast, 
            InvertType.OpposeMaster
        );
        public static final TalonConstants hoodConstants = new TalonConstants(
            hoodID, 
            talonCurrentLimit.supplyCurLim40, 
            NeutralMode.Brake, 
            InvertType.InvertMotorOutput
        );
        public static final TalonConstants turretConstants = new TalonConstants(
            0, 
            talonCurrentLimit.supplyCurLim40, 
            NeutralMode.Brake, 
            InvertType.None
        ); //might need to change invert type
        
        /* Mechanism properties */
        public static final double shooterGearRatio = (1/1);
        public static final double hoodGearRatio = 18./42;
        public static final double hoodAngleOffset = 10.;
        public static final int hoodEncoderCountsPerRev = 8192;
        public static final double turretGearRatio = 0;
        public static final double hoodLowLimit = 10;
        public static final double hoodHighLimit = 90;
        public static final double turretLowLimit = 0;
        public static final double turretHighLimit = 0;

        /* Shooter characterization and tuning values */
        public static final double shootKs = 0.75347;
        public static final double shootKv = 0.11309;
        public static final double shootKa = 0.0073;
        public static final PIDGains shooterPID = new PIDGains(0.15, 0, 1.0, 1023.0 / Conversions.RPMToFalcon(5700, shooterGearRatio));//0.046976); // might need to be changed
        public static final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(shootKs, shootKv, shootKa);
        public static final PIDGains hoodPID = new PIDGains(0.05, 0,
         0.001, 0.12); //contains up FF
        public static final double hoodDownFF = -0.05;
        public static final double hoodControllerToleranceDegrees = 0.5;
        public static final PIDGains turretPID = new PIDGains(0, 0, 0, 0);//Definetly needs to be changed  

        /* Shooter Calibration Values */
        public static final double[][] shooterMap = 
        // {distance (m), shooter speed (RPM), shooter angle (degrees from horiz)}
        {
            // {1.84, 2200, 14.6},
            // {1.95, 2400, 21},
            // {2.11, 2275, 22},
            // {2.84, 2500, 25},
            // {3.75, 2700, 27},
            // {4.19, 2900, 29}, //tentative - TODO
            // {4.71, 3200, 32}
            // //{-1, 3300, 35}    
            
            {1.18 + Units.feetToMeters(4)/2, 2250, 16.3},
            {1.75 + Units.feetToMeters(4)/2, 2200, 18.5},
            // {2.06 + Units.feetToMeters(4)/2, 2300, 20.5},
            // {2.43 + Units.feetToMeters(4)/2, 2400, 24.0},
            {2.16 + Units.feetToMeters(4)/2, 2250, 23},
            {2.61 + Units.feetToMeters(4)/2, 2425, 24},
            {3.17 + Units.feetToMeters(4)/2, 2500, 25},
            {3.64 + Units.feetToMeters(4)/2, 2700, 26.5},
            {4.20 + Units.feetToMeters(4)/2, 3100, 28.9}
        };

        public static final double[] shooterLowMap = 
        // distance, shooter speed, shooter angle
        {1.83, 900, 14.00};

    }



    /* Container for vision subsystem constants */
    public static final class Vision {
        public static final double goalHeight = Units.inchesToMeters(101.375);
        public static final double goalDiameter = Units.feetToMeters(4);

        public static final double limelightHeight = Units.inchesToMeters(21.0);
        public static final Rotation2d limelightAngle = Rotation2d.fromDegrees(48);
    }



    /* Container for climber subsystem constants */
    public static final class Climber {

        /* Motor and device IDs */
        private static final int climberMotorID = 15;
        public static final int ClimberSolenoidForwardChannel = 3;
        public static final int ClimberSolenoidReverseChannel = 2;
        public static final int climberEncoderAbsoluteChannel = -1;
        public static final int[] climberEncoderRelativeChannels = {6, 7};
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
        public static final double climberGearRatio = 12.;
        public static final double extendedCounts = 0;
        public static final double retractedCounts = 0;
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
        public static final TalonConstants intakeMotorConstants = new TalonConstants(
            intakeID, 
            talonCurrentLimit.supplyCurLim40, 
            NeutralMode.Brake, 
            InvertType.InvertMotorOutput
        );
        public static final TalonConstants indexMotorConstants = new TalonConstants(
            indexID, 
            talonCurrentLimit.supplyCurLim40, 
            NeutralMode.Brake, 
            InvertType.None
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

        /* Autonomous routine movement constraints */
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
        public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(Constants.Swerve.swerveKinematics);

        public static final double thetaKP = 5.;
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = Swerve.kThetaControllerConstraints;

        /* Autonomous waypoints */
        public static final Pose2d startPos = new Pose2d(7.606, 2.974, new Rotation2d(-1.894));
         //Starting Positions
            //Right
            //new Pose2d(7.606, 2.974, new Rotation2d(-1.894)),   //0 Defalt bottom starting position
            //new Pose2d(6.601, 2.546, new Rotation2d(-2.283)),   //1 Right bottom starting position
            //new Pose2d(8.439, 1.876, new Rotation2d(-1.561)),   //2 Left bottom starting position
            //Left
            //new Pose2d(7.103, 4.871, new Rotation2d(2.742)),    //Defalt top starting position
            //new Pose2d(5.962, 3.958, new Rotation2d(3.141)),    //Right top starting position
            //new Pose2d(6.764, 5.712, new Rotation2d(2.035)),    //Left top starting position
        public static final SwerveTrajectoryWaypoint[] rightPoints = 
        {
            new SwerveTrajectoryWaypoint(7.608, 0.929, -1.571, -1.571), //part1 end - ball1 
            new SwerveTrajectoryWaypoint(7.608, 0.929, -1.571, 2.912), //part2 start - ball1
            new SwerveTrajectoryWaypoint(5.452, 1.534, 2.989, 2.989), //part2 interior - ball2
            new SwerveTrajectoryWaypoint(1.483, 1.557, -2.452, 2.286), //part2 interior - ball3
            new SwerveTrajectoryWaypoint(3.496, 2.619, 0.318 + Math.PI, 0.318), //part2 end - shoot pos
            new SwerveTrajectoryWaypoint(3.496, 2.619, 0.314, 0), //part3 start - shoot pos
            new SwerveTrajectoryWaypoint(4.636, 5.682, 0.983, 0), //part3 interior - ball4
            new SwerveTrajectoryWaypoint(5.626, 6.013, -0.628, 0) //part3 end - final shoot pos
        };

        public static final SwerveTrajectoryWaypoint[] leftPoints = 
        {
            new SwerveTrajectoryWaypoint(5.245, 5.955, 2.618, 2.618),
            new SwerveTrajectoryWaypoint(4.213, 4.564, -2.147, -2.147),
            new SwerveTrajectoryWaypoint(1.685, 1.697, -2.374, -2.374),
            new SwerveTrajectoryWaypoint(2.297, 1.726, 0.209, 0.209),
            new SwerveTrajectoryWaypoint(5.464, 1.954, 0, 0),
            new SwerveTrajectoryWaypoint(6.198, 1.997, -2.356, -2.356),
            new SwerveTrajectoryWaypoint(7.559, 1, -1.571, -1.571)

            // new Pose2d(5.245, 5.955, new Rotation2d(2.622)),
            // new Pose2d(4.213, 4.564, new Rotation2d(-2.141)),
            // new Pose2d(1.685, 1.697, new Rotation2d(-2.37)),
            // new Pose2d(2.297, 1.726, new Rotation2d(0.216)),
            // new Pose2d(5.464, 1.954, new Rotation2d(0.008)),
            // new Pose2d(7.559, 1, new Rotation2d(-1.571))
        };

        public static final SwerveTrajectoryWaypoint[] optimizedRightPoints =
        {
            new SwerveTrajectoryWaypoint(7.608, 0.929, -1.571, -1.571), //part1 end got ball 1
            new SwerveTrajectoryWaypoint(7.608, 0.929, -1.571, 2.597),  //part2 start go to ball 2            
            new SwerveTrajectoryWaypoint(5.355, 2.026, -2.975, 2.379),  //part2 end got ball 2
            new SwerveTrajectoryWaypoint(5.355, 2.026, -2.975, -2.975), //part3 start go ball 3 and 4
            new SwerveTrajectoryWaypoint(1.467, 1.512, -2.744, -3.005),      //part3 mid get ball 3 and 4
            new SwerveTrajectoryWaypoint(1.467, 1.512, -2.744, 0.724),      //part4 start get ball 3 and 4
            new SwerveTrajectoryWaypoint(3.615, 2.952, -2.938, 0.533)     //part4 end go to shoot
        };
    }
    
}
