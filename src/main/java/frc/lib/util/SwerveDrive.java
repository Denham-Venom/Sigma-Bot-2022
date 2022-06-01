// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDrive {
    // Device Refs
    public SwerveModule[] swerveMods;
    public PigeonIMU gyro;
    public Limelight limelight;

    // State Variables
    public SwerveDriveOdometry swerveOdometry;
    private boolean fieldRelative = true;
    public Translation2d goalRelTranslation = new Translation2d();
    private ShuffleboardTab Drivers = Shuffleboard.getTab("Drivers");
    private NetworkTableEntry swerveReady = Drivers.add("Swerve Ready" , false).getEntry();
    private NetworkTableEntry fieldRelEntry = Drivers.add("Field Rel" , true).getEntry();
    
    public SwerveDrive(SwerveModule[] mSwerveMods, PigeonIMU mGyro, SwerveDriveOdometry mSwerveOdometry) {
        swerveMods = mSwerveMods;

        gyro = mGyro;
        gyro.configFactoryDefault();
        zeroGyro();

        swerveOdometry = mSwerveOdometry;
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());
    }

    public SwerveDrive(SwerveModule[] mSwerveMods, PigeonIMU mGyro, SwerveDriveOdometry mSwerveOdometry, Limelight limelight) {
        swerveMods = mSwerveMods;

        gyro = mGyro;
        gyro.configFactoryDefault();
        zeroGyro();

        swerveOdometry = mSwerveOdometry;
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());
        this.limelight = limelight;
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

        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
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
     * Zero robot gyro, resetting the orientation the robot uses for field relative control.  
     */
    public void zeroGyro(){
        gyro.setYaw(0);
    }

    /**
     * Gets the states of the swerve modules.
     * @return 1D array of length 4 containing SwerveModuleState objects 
     * for each module at the position corresponding to the number of the module.
     */
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : swerveMods){
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
        for(var mod : swerveMods){
            mod.setNeutral(neutral);
        }
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
    
}
