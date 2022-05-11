package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.newWpilibUtils.NewSwervePoseEstimator;
import frc.lib.newWpilibUtils.NewTimeInterpolatableBuffer;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase{
    public NewSwervePoseEstimator sEstimator;
    public NewTimeInterpolatableBuffer<Double> gyroYawBuffer = NewTimeInterpolatableBuffer.createDoubleBuffer(1.0);
    public Pose2d visionPose = new Pose2d();

    public PoseEstimator(){
        sEstimator = new NewSwervePoseEstimator(
            new Rotation2d(), 
            new Pose2d(), 
            Constants.Swerve.swerveKinematics, 
            Constants.PoseEstimator.stateStdDevs, 
            Constants.PoseEstimator.gyroStdDevs, 
            Constants.PoseEstimator.visionStdDevs
        );

        SmartDashboard.putNumber("LLAngle", 0.0);
        SmartDashboard.putNumber("LLTY", 0.0);
        SmartDashboard.putNumber("gyroYaw", 0.0);

        
    }

    /** Check if this returns true before using {@link #updateVision()} 
     * @return If time buffers are !null */
    public boolean readyToUpdateVision(){
        return (gyroYawBuffer.getSample(0).isPresent());
    }

    /** Update estimator with Swerve States and Gyro Yaw data.
     * Needs to be updated every loop. */
    public void updateSwerve(Rotation2d gyroAngle, SwerveModuleState[] states, Translation2d robot_velocity){
        sEstimator.update(gyroAngle, states);
        gyroYawBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle.getRadians());
    }

    /** Update estimator with vision data. 
     *  Should only be updated when target is visible.
     * @param LLlatency seconds */
    public void updateVision(Translation2d LLdistance, Rotation2d LLangle, double LLlatency){
        double timeStamp = Timer.getFPGATimestamp() - LLlatency;

        sEstimator.addVisionMeasurement(
            getFieldToRobotPose(
                LLdistance, 
                LLangle,  
                new Rotation2d(gyroYawBuffer.getSample(timeStamp).get())), 
            timeStamp
        );
    }

    /** Field to Robot Transforms */ 
    private Pose2d getFieldToRobotPose(Translation2d LLdistance, Rotation2d LLangle, Rotation2d gyroYaw){
        /* Field to Target */
        Pose2d fieldToTarget = Constants.Vision.goalPose;

        /* Field to Target to LL */
        Transform2d targetToLL = 
            new Transform2d(
                LLdistance.rotateBy(LLangle.plus(Constants.Vision.shooterAngle).plus(gyroYaw)).unaryMinus(),
                LLangle.plus(Constants.Vision.shooterAngle).plus(gyroYaw)
            );
        Pose2d fieldToLL = fieldToTarget.transformBy(targetToLL);

        /* Field to Target to LL to Shooter */
        Transform2d LLToShooter = new Transform2d(Constants.Vision.shooterToLLOffset, LLangle);
        Pose2d fieldToShooter = fieldToLL.transformBy(LLToShooter.inverse());

        /* Field to Target to LL to Shooter to Robot */
        Transform2d shooterToRobot = new Transform2d(Constants.Vision.robotToShooterOffset, Constants.Vision.shooterAngle);
        Pose2d fieldToRobot = fieldToShooter.transformBy(shooterToRobot.inverse());
        visionPose = fieldToRobot;      
        return fieldToRobot;
    }

    /** @return Distance in meters from shooter to alliance target */
    public double getShooterDistanceToTarget(){
        return getShooterToTargetPose().getTranslation().getNorm();
    }

    /** @return Angle from shooter to alliance target */
    public Rotation2d getShooterAngleToTarget(){
        Pose2d diffPose = getShooterToTargetPose();
        return new Rotation2d(Math.atan2(diffPose.getY(), diffPose.getX()));
    }

    /** @return Pose2d of the Target Pose relative to the Shooter Pose */
    private Pose2d getShooterToTargetPose(){
        /* Robot to Turret Center */
        Transform2d turretToRobot = new Transform2d(Constants.Vision.robotToShooterOffset, new Rotation2d());
        Pose2d turretPose = sEstimator.getEstimatedPosition().transformBy(turretToRobot);
        return Constants.Vision.goalPose.relativeTo(turretPose);
    }

    

    /* */
    /**@return Median filtered distance to Target in meters */
    public Translation2d getDistance(Rotation2d ty){
        double heightDifference = Constants.Vision.goalHeight - Constants.Vision.limelightHeight;
        Rotation2d combinedAngle = Constants.Vision.limelightAngle.plus(ty);
        return new Translation2d((heightDifference / combinedAngle.getTan()), 0);
    }
    
    @Override
    public void periodic(){
        Translation2d distance = getDistance(Rotation2d.fromDegrees(SmartDashboard.getNumber("LLTY", 0.0)));
        Pose2d test = getFieldToRobotPose(
            distance, 
            Rotation2d.fromDegrees(SmartDashboard.getNumber("LLAngle", 0.0)), 
            Rotation2d.fromDegrees(SmartDashboard.getNumber("gyroYaw", 0.0))
        );
        System.out.println(test);

    }
}