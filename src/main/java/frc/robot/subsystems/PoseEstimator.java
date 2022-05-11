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

    public PoseEstimator(){
        sEstimator = new NewSwervePoseEstimator(
            new Rotation2d(), 
            new Pose2d(), 
            Constants.Swerve.swerveKinematics, 
            Constants.PoseEstimator.stateStdDevs, 
            Constants.PoseEstimator.gyroStdDevs, 
            Constants.PoseEstimator.visionStdDevs
        );
    }

    /** Check if this returns true before using {@link #updateVision()} 
     * @return If time buffers are !null */
    public boolean readyToUpdateVision(){
        return (gyroYawBuffer.getSample(0).isPresent());
    }

    /** Update estimator with Swerve States and Gyro Yaw data.
     * Needs to be updated every loop. */
    public void updateSwerve(Rotation2d gyroAngle, SwerveModuleState[] states){
        sEstimator.update(gyroAngle, states);
        gyroYawBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle.getRadians());
    }

    /** Update estimator with vision data. 
     *  Should only be updated when target is visible.
     * @param LLlatency seconds */
    public void updateVision(Translation2d targetDistance, Rotation2d LLTX, double LLlatency){
        double timeStamp = Timer.getFPGATimestamp() - LLlatency;

        sEstimator.addVisionMeasurement(
            getFieldToRobotPose2d(
                targetDistance, 
                LLTX,  
                new Rotation2d(gyroYawBuffer.getSample(timeStamp).get())), 
            timeStamp
        );
    }

    /** Field to Robot Transforms */ 
    private Pose2d getFieldToRobotPose2d(Translation2d targetDistance, Rotation2d LLTX, Rotation2d gyroYaw){
        /* Field to Target */
        Pose2d fieldToTarget = Constants.Vision.goalPose;

        /* Field to Target to LL */
        Transform2d targetToLL = 
            new Transform2d(
                targetDistance.rotateBy(LLTX.plus(Constants.Vision.shooterAngle).plus(gyroYaw)).unaryMinus(),
                LLTX.plus(Constants.Vision.shooterAngle).plus(gyroYaw)
            );
        Pose2d fieldToLL = fieldToTarget.transformBy(targetToLL);

        /* Field to Target to LL to Shooter */
        Transform2d LLToShooter = new Transform2d(Constants.Vision.shooterToLLOffset, LLTX);
        Pose2d fieldToShooter = fieldToLL.transformBy(LLToShooter.inverse());

        /* Field to Target to LL to Shooter to Robot */
        Transform2d shooterToRobot = new Transform2d(Constants.Vision.robotToShooterOffset, Constants.Vision.shooterAngle);
        Pose2d fieldToRobot = fieldToShooter.transformBy(shooterToRobot.inverse());

        return fieldToRobot;
    }

    /** @return Distance in meters from shooter to alliance target */
    public double getShooterDistanceToTarget(){
        return getShooterToTargetPose().getTranslation().getNorm();
    }

    /** @return Pose2d of the Target Pose relative to the Shooter Pose */
    private Pose2d getShooterToTargetPose(){
        /* Robot to Shooter Center */
        Transform2d shooterToRobot = new Transform2d(Constants.Vision.robotToShooterOffset, new Rotation2d());
        Pose2d shooterPose = sEstimator.getEstimatedPosition().transformBy(shooterToRobot);
        return Constants.Vision.goalPose.relativeTo(shooterPose);
    }

    /** @return Angle Robot needs to rotate to, to face shooter (back of robot) to target */
    public Rotation2d getRobotTargetYaw(){
        Pose2d diffPose = Constants.Vision.goalPose.relativeTo(sEstimator.getEstimatedPosition());
        Rotation2d targetAngle = new Rotation2d(Math.atan2(diffPose.getY(), diffPose.getX()));
        targetAngle = targetAngle.plus(Constants.Vision.shooterAngle);
        return sEstimator.getEstimatedPosition().getRotation().plus(targetAngle);
    } 
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("robotX", sEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("robotY", sEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("robotHeading", sEstimator.getEstimatedPosition().getRotation().getRadians());
    }
}