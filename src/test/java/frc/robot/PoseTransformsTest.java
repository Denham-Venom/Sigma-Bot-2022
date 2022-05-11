package frc.robot;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.newWpilibUtils.Rotation3d;
import frc.lib.newWpilibUtils.Translation3d;

public class PoseTransformsTest {
    Pose2d robotPose;

    Rotation2d LLTY = Rotation2d.fromDegrees(-15);
    Translation2d targetDistance = getDistance(LLTY);
    Rotation2d LLTX = Rotation2d.fromDegrees(90);
    Rotation2d gyroYaw = Rotation2d.fromDegrees(0);

    @Test
    public void testing(){
        print(getFieldToRobotPose2d(targetDistance, LLTX, gyroYaw));
        // print(getFieldToRobotPose3d(LLTY, LLTX, gyroYaw));
        print(getRobotTargetAngle());
        print(getShooterToTargetPose());
     }

    public void print(Object printing) {
        System.out.println(printing);
    }

    /* */
    /**@return Median filtered distance to Target in meters */
    public Translation2d getDistance(Rotation2d ty){
        double heightDifference = Constants.Vision.goalHeight - Constants.Vision.limelightHeight;
        Rotation2d combinedAngle = Constants.Vision.limelightAngle.plus(ty);
        return new Translation2d((heightDifference / combinedAngle.getTan()), 0);
    }

    /** Field to Robot Transforms */ 
    public Pose2d getFieldToRobotPose2d(Translation2d targetDistance, Rotation2d LLTX, Rotation2d gyroYaw){
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

        robotPose = fieldToRobot;
        return fieldToRobot;
    }

    public Pose2d getFieldToRobotPose3d(Rotation2d robotHeading, Rotation2d tx, Rotation2d ty){
        Rotation2d robotZero2Goal = robotHeading.plus(Constants.Vision.shooterAngle).minus(Rotation2d.fromDegrees(180));

        Rotation3d cameraOrientation = new Rotation3d(0, Constants.Vision.limelightAngle.getRadians(), 0);
        Rotation3d targetOrientation = new Rotation3d(0, ty.getRadians(), tx.getRadians()).plus(cameraOrientation);

        double tan_ty = new Rotation2d(targetOrientation.getY()).getTan();
        double tan_tx = new Rotation2d(targetOrientation.getZ()).getTan();
        double distnace = (Constants.Vision.goalHeight - Constants.Vision.limelightHeight) * Math.sqrt(1 + tan_tx * tan_tx + tan_ty * tan_ty)/tan_ty;
        Translation3d distance3D = new Translation3d(distnace, targetOrientation);

        Translation2d LLpos = new Translation2d(distance3D.getX(),distance3D.getY()).rotateBy(robotZero2Goal);
        Translation2d shooterTranslation = Constants.Vision.robotToShooterOffset.rotateBy(robotHeading); 
        Translation2d LLTranslation = Constants.Vision.shooterToLLOffset.rotateBy(robotHeading).rotateBy(Constants.Vision.shooterAngle); 

        Translation2d fieldToRobotTranslation = LLpos.minus(LLTranslation).minus(shooterTranslation).plus(Constants.Vision.goalPose.getTranslation());

        robotPose = new Pose2d(fieldToRobotTranslation, robotHeading);
        return new Pose2d(fieldToRobotTranslation, robotHeading);
    }

    /** @return Distance in meters from shooter to alliance target */
    public double getShooterDistanceToTarget(){
        return getShooterToTargetPose().getTranslation().getNorm();
    }

    /** @return Pose2d of the Target Pose relative to the Shooter Pose */
    private Pose2d getShooterToTargetPose(){
        /* Robot to Shooter Center */
        Transform2d shooterToRobot = new Transform2d(Constants.Vision.robotToShooterOffset, new Rotation2d());
        Pose2d shooterPose = robotPose.transformBy(shooterToRobot);
        return Constants.Vision.goalPose.relativeTo(shooterPose);
    }

    /** @return Angle Robot needs to rotate to, to face shooter (back of robot) to target */
    public Rotation2d getRobotTargetAngle(){
        Pose2d diffPose = Constants.Vision.goalPose.relativeTo(robotPose);
        Rotation2d targetAngle = new Rotation2d(Math.atan2(diffPose.getY(), diffPose.getX()));
        targetAngle = targetAngle.plus(Constants.Vision.shooterAngle);
        return robotPose.getRotation().plus(targetAngle);
    }    
}