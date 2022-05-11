package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    private PoseEstimator m_poseEstimator;
    public SwerveDriveOdometry autoOdometry;
    private Limelight limelight;

    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    // State Variables
    private int currentNeutral = 0;
    private boolean isLowGear = true;
    private boolean fieldRelative = true;
    
    // Network Table Variables
    private static ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
    private NetworkTableEntry turnP = tuning.add("Turn P", 0).getEntry();
    private NetworkTableEntry turnI = tuning.add("Turn I", 0).getEntry();
    private NetworkTableEntry turnD = tuning.add("Turn D", 0).getEntry();
    private NetworkTableEntry turnTol = tuning.add("Turn Tol", 0).getEntry();
    private NetworkTableEntry tuneSwerve = tuning.add("Tune Swerve", false).getEntry();
    public static NetworkTableEntry transRateLimiting = tuning.add("Trans Rate Limiting", 0).getEntry();
    public static NetworkTableEntry turnRateLimiting = tuning.add("Turn Rate Limiting", 0).getEntry();

    private ShuffleboardTab Drivers = Shuffleboard.getTab("Drivers");
    private NetworkTableEntry swerveReady = Drivers.add("Swerve Ready" , false).getEntry();
    private NetworkTableEntry fieldRelEntry = Drivers.add("Field Rel" , true).getEntry();

    private PIDController thetaController;
    private double turnTolVal = Constants.Swerve.thetaTolerance;
    private Translation2d inputTranslation = new Translation2d();

    /**
     * Create a new swerve subsystem.
     * @param m_Vision The limelight to use for target acquisition.
     */
    public Swerve(PoseEstimator m_poseEstimator, Vision m_Vision) {
        this.m_poseEstimator = m_poseEstimator;
        limelight = m_Vision.getLimelight();

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        autoOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());
        thetaController = new PIDController(Constants.Swerve.thetaKP, Constants.Swerve.thetaKI, Constants.Swerve.thetaKD);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }

    /**
     * Switches between high and low gear (soft switch).
     * @return True if now in low gear, false if in high.
     */
    public boolean switchLowHighGear() {
        if(isLowGear) {
            isLowGear = false;
        } else {
            isLowGear = true;
        }
        return isLowGear;
    }

    /**
     * Get gear of robot.
     * @return Double value representing multiplier to use for inputted speed 
     * values. 
     */
    public double gethighLowGear() {
        if(isLowGear) {
            return Constants.Swerve.lowGearValue;
        } else {
            return Constants.Swerve.highGearValue;
        }
    }

    /**
     * Function used to move the swerve drive. Formerly took in fieldRelative parameter 
     * representing whether the translational velocity was relative to current orientation or 
     * to current orientation with respect to the field. This was changed for favor of this 
     * being state information managed by the swerve subsystem.
     * @param translation Container for x and y translational velocity in meters per seconds
     * @param rotation Double value for rotational velocity in rads/sec
     * @param isOpenLoop Whether to use open loop or feedback control to achieve drive control
     */
    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getYaw()
                )
                : new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    /**
     * Gets pose of drivetrain using auto only odometry.
     * @return Pose2d pose with x and y in meters
     */
    public Pose2d getAutoPose() {
        return autoOdometry.getPoseMeters();
    }

    /**
     * Resets auto odometry of the drivetrain to a specified pose
     * @param pose The target pose to set robot odom to with x and y in meters
     */
    public void resetAutoOdometry(Pose2d pose) {
        autoOdometry.resetPosition(pose, pose.getRotation());
    }

    /**
     * Sets the neutral mode for the sweve modules.
     * @param neutral CTRE Phoenix Library NeutralMode enum value.
     */
    public void setNeutral(NeutralMode neutral){
        for(SwerveModule mod : mSwerveMods){
            mod.setNeutral(neutral);
        }
    }

    /**
     * Gets the states of the swerve modules.
     * @return 1D array of length 4 containing SwerveModuleState objects 
     * for each module at the position corresponding to the number of the module.
     */
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Zero robot gyro, resetting the orientation the robot uses for field relative control.  
     */
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void setGyro(double input){
        gyro.setYaw(input);
    }

    /**
     * Get yaw of robot, which is the direction the robot is facing.
     * @return Rotation2d representation of yaw.
     */
    public Rotation2d getYaw() {
        double yaw = gyro.getYaw();
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }
    
    /**
     * Toggles whether the robot uses field relative (default true) control, 
     * which determines whether the robot moves translationally relative to 
     * its current orientation or relative to what it thinks the fields orientation is.
     * @return The new value of field relative
     */
    public boolean toggleFieldRelative() {
        fieldRelative = !fieldRelative;
        fieldRelEntry.setBoolean(fieldRelative);
        return fieldRelative;
    }

    public void updateTranslationInput(Translation2d translation) {
        inputTranslation = translation;
    }

    @Override
    public void periodic() {
        m_poseEstimator.updateSwerve(getYaw(), getStates());
        autoOdometry.update(getYaw(), getStates());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        }

        if(Constants.Swerve.coastOnDisable){
            if (DriverStation.isEnabled() && currentNeutral == 1){
                setNeutral(NeutralMode.Brake);
                currentNeutral = 0;
            }
            else if (DriverStation.isDisabled() && currentNeutral == 0){
                setNeutral(NeutralMode.Coast);
                currentNeutral = 1;
            }
        }

        switch(States.shooterState){
            case preShoot:
                double targetYawAngle = limelight.hasTarget() ? 
                    getYaw().getRadians() + limelight.getTx().getRadians() + Constants.Vision.shooterAngle.getRadians() : 
                    m_poseEstimator.getRobotTargetYaw().getRadians();

                swerveReady.setBoolean(Math.abs(targetYawAngle - getYaw().getRadians()) < turnTolVal);
                drive(
                    inputTranslation, 
                    thetaController.calculate(getYaw().getRadians(), targetYawAngle), 
                    false
                );
                break;
            default:
                swerveReady.setBoolean(false);
                break;
        }
        
        if(Constants.tuningMode) {
            if (tuneSwerve.getBoolean(false)) {
                double p = turnP.getDouble(0), i = turnI.getDouble(0), d = turnD.getDouble(0);
                if(p != thetaController.getP()
                        || i != thetaController.getI()
                        || d != thetaController.getD()) {
                    thetaController.setPID(p, i, d);   
                }

                double tol = turnTol.getDouble(0), velTol = turnTol.getDouble(0);
                if(tol != turnTolVal) {
                    turnTolVal = tol;
                    thetaController.setTolerance(turnTolVal);
                }
            }
        }
    }
}