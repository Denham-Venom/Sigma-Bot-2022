package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.SwerveModule;
import frc.robot.commands.TeleopSwerve;

public class Swerve extends SubsystemBase {

    // Device Refs
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    private Limelight limelight;

    // State Variables
    public SwerveDriveOdometry swerveOdometry;
    private int currentNeutral = 0;
    private double thetaOut;
    private boolean isLowGear = true;
    private boolean fieldRelative = true;
    public Translation2d goalRelTranslation = new Translation2d();
    
    // Network Table Variables
    public NetworkTable table;
    public NetworkTableInstance inst;
    public NetworkTableEntry xEntry;
    public NetworkTableEntry yEntry;
    public NetworkTableEntry headingEntry;
    private static ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
    private NetworkTableEntry turnP = tuning.add("Turn P", 0).getEntry();
    private NetworkTableEntry turnI = tuning.add("Turn I", 0).getEntry();
    private NetworkTableEntry turnD = tuning.add("Turn D", 0).getEntry();
    private NetworkTableEntry turnTol = tuning.add("Turn Tol", 0).getEntry();
    private NetworkTableEntry tuneSwerve = tuning.add("Tune Swerve", false).getEntry();
    public static NetworkTableEntry transRateLimiting = tuning.add("Trans Rate Limiting", 0).getEntry();
    public static NetworkTableEntry turnRateLimiting = tuning.add("Turn Rate Limiting", 0).getEntry();
    private double turnTolVal = Constants.Swerve.thetaTolerance;
    private ShuffleboardTab Drivers = Shuffleboard.getTab("Drivers");
    private NetworkTableEntry swerveReady = Drivers.add("Swerve Ready" , false).getEntry();
    private NetworkTableEntry fieldRelEntry = Drivers.add("Field Rel" , true).getEntry();

    // Feedback Controllers
    public final PIDController xController = new PIDController(
        Constants.Swerve.xKP, 
        Constants.Swerve.xKI, 
        Constants.Swerve.xKD
    );
    public final PIDController yController = new PIDController(
        Constants.Swerve.yKP, 
        Constants.Swerve.yKI, 
        Constants.Swerve.yKD
    );
    public final PIDController thetaController = new PIDController(
        Constants.Swerve.thetaKP, 
        Constants.Swerve.thetaKI, 
        Constants.Swerve.thetaKD
    );
    


    /**
     * Create a new swerve subsystem.
     * @param m_Vision The limelight to use for target acquisition.
     */
    public Swerve(Vision m_Vision) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(turnTolVal);        
        thetaController.setSetpoint(0);
        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        limelight = m_Vision.getLimelight();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Live_Dashboard");
        xEntry = table.getEntry("robotX");
        yEntry = table.getEntry("robotY");
        headingEntry = table.getEntry("robotHeading");

        SmartDashboard.putNumber("robotX_swerve_odo", 0);
        SmartDashboard.putNumber("robotY_swerve_odo", 0);
        SmartDashboard.putNumber("robotAng_swerve_odo", 0);
    }

    //TODO - make high/low gear mangaged internally in swerve, not in teleopswerve command
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
     * Gets pose of drivetrain
     * @return Pose2d pose with x and y in meters
     */
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Resets odometry of the drivetrain to a specified pose
     * @param pose The target pose to set robot odom to with x and y in meters
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    /**
     * Sets the neutral mode for the sweve modules.
     * @param neutral CTRE Phoenix Library NeutralMode enum value.
     */
    public void setNeutral(NeutralMode neutral){
        for(var mod : mSwerveMods){
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

    /**
     * Get yaw of robot, which is the direction the robot is facing.
     * @return Rotation2d representation of yaw.
     */
    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    /**
     * Finds the angle to the field vision targets from the robots current orientation.
     * @return Rotation2d representing the rotation required to center on the target.
     */
    public Rotation2d getAngleToTarget(){
        Rotation2d angle;
        Pose2d robotPose = getPose();
        Translation2d centerGoal = new Translation2d(8.218, 4.115);
        Translation2d goalVector = robotPose.getTranslation().minus(centerGoal);
        //angle = new Rotation2d(goalVector.getX(), goalVector.getY()).minus(robotPose.getRotation());
        angle = new Rotation2d(goalVector.getX(), goalVector.getY());
        return angle;
    }

    public void setTargetRel(){
        Translation2d llx = limelight.getDistance(); //t2d with x as dist from robot to target
        Pose2d curPos = getPose(); //cur robot odom
        Rotation2d curRot = curPos.getRotation(); //rot from odom
        Rotation2d curLLRot = curRot.plus(new Rotation2d(Math.PI)); //rot of LL (back of robot) from odom
        Translation2d llxFR = llx.rotateBy(curLLRot); //rot LL dist from robot to goal vector to be pointed in the correct direction relative to field coords
        goalRelTranslation = curPos.getTranslation().plus(llxFR); //get final goal pos by adding vector from field origin to robot to vector from robot to target
        //goalRelTranslation = limelight.getDistance().rotateBy(getPose().getRotation().plus(Rotation2d.fromDegrees(180))).plus(getPose().getTranslation()); //store value
    }

    public void setTarget(){
        Translation2d llDistance = limelight.getDistance().plus(new Translation2d(Constants.Vision.goalDiameter/2, 0));
        Pose2d curPos = getPose();
        Rotation2d llx = limelight.getTx();
        Rotation2d opHeading = curPos.getRotation().plus(new Rotation2d(Math.PI));//rotate poseheading to reflect direction of limelight
        Rotation2d goalAngle = opHeading.plus(llx);
        Translation2d goalVector = llDistance.rotateBy(goalAngle);
        goalRelTranslation = curPos.getTranslation().plus(goalVector);
    }

    public Translation2d getTarget(){
        return goalRelTranslation.minus(getPose().getTranslation());
    }

    public Rotation2d getAngleToTargetRel(){
        Rotation2d angle;
        Pose2d robotPose = getPose();
        Translation2d goalVector = goalRelTranslation.minus(robotPose.getTranslation());
        //angle = new Rotation2d(goalVector.getX(), goalVector.getY()).minus(robotPose.getRotation());
        angle = new Rotation2d(goalVector.getX(), goalVector.getY()).plus(Rotation2d.fromDegrees(180)).minus(robotPose.getRotation());
        SmartDashboard.putNumber("estimated Angle to Target", angle.getDegrees());
        return angle;
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

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getStates());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        //thetaOut = limelight.hasTarget()? thetaController.calculate(limelight.getTx().getDegrees()): thetaController.calculate(getAngleToTarget().getDegrees());
        double measurement = limelight.getTx().getRadians();
        //thetaOut = limelight.hasTarget()? thetaController.calculate(measurement): thetaController.calculate(getAngleToTargetRel().getRadians());
        //thetaOut = thetaController.calculate(limelight.getTx().getRadians()); //Constants.Swerve.thetaTolerance >= limelight.getTx().getDegrees() && limelight.getTx().getDegrees() >= -Constants.Swerve.thetaTolerance ? 0 : -thetaController.calculate(limelight.getTx().getRadians());
        //thetaOut = thetaController.calculate(measurement);
        SmartDashboard.putBoolean("limelight has Target", limelight.hasTarget());
        SmartDashboard.putNumber("theta out", thetaOut);
        SmartDashboard.putNumber("limelight out", limelight.getTx().getDegrees());
        //if(Math.abs(limelight.getTx().getDegrees()) <= Constants.Swerve.thetaTolerance) setTargetRel();
        if(limelight.hasTarget()) {
            setTarget();
            thetaOut = thetaController.calculate(measurement);
            if(Math.abs(measurement) <= turnTolVal) {
                thetaOut = 0;
                //setTargetRel();
                swerveReady.setBoolean(true);
                
            }
            else {
                swerveReady.setBoolean(false);
            }
        } else {
            thetaOut = thetaController.calculate(getAngleToTargetRel().getRadians());
            swerveReady.setBoolean(false);
        }


        switch(States.shooterState){    
            case preShoot:
                Translation2d t = ((TeleopSwerve)this.getDefaultCommand()).getTranslation2d();
                this.drive(t, thetaOut, false);
                break;
            default:
                break;
        }

        if(Constants.Swerve.coastOnDisable){
            if (DriverStation.isEnabled() && currentNeutral == 1){
                setNeutral(NeutralMode.Brake);
                currentNeutral = 0;
            }
            else if (DriverStation.isDisabled() && currentNeutral == 0){
                //setNeutral(NeutralMode.Coast); TODO change back
                currentNeutral = 1;
            }
        }
        

        double x = Units.metersToFeet(swerveOdometry.getPoseMeters().getX());
        double y = Units.metersToFeet(swerveOdometry.getPoseMeters().getY());
        double ang = swerveOdometry.getPoseMeters().getRotation().getRadians();
        xEntry.setDouble(x);
        yEntry.setDouble(y);
        headingEntry.setDouble(ang);

        //SmartDashboard.putNumber("robotX_swerve_odo", x);
        //SmartDashboard.putNumber("robotY_swerve_odo", y);
        //SmartDashboard.putNumber("robotAng_swerve_odo", ang);

        if(Constants.tuningMode && tuneSwerve.getBoolean(false)) {
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