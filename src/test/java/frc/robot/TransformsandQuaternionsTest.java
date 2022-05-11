package frc.robot;

import java.util.Random;

import frc.lib.newWpilibUtils.Rotation3d;
import frc.lib.newWpilibUtils.Translation3d;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TransformsandQuaternionsTest {
    Random rand = new Random();
    int max = 15;
    int min = -15;

    @Test
    public void testing() {
        for (int i = 0; i < 10; i++) {
            int tY = -15;
            int tX = 20;

            // int tY = rand.nextInt((max - min) + 1) + min;
            // int tX = rand.nextInt((max - min) + 1) + min;

            Translation2d targetDistance = getDistance(Rotation2d.fromDegrees(tY));
            Rotation2d LLTX = Rotation2d.fromDegrees(tX);
            Rotation2d gyroYaw = Rotation2d.fromDegrees(30);

            Pose2d local2d = getFieldToRobotPose2d(targetDistance, LLTX, gyroYaw);
            Pose2d local3d = getFieldToRobotPose3d(gyroYaw, LLTX, Rotation2d.fromDegrees(tY));
            double percentdiffernce = (local2d.getTranslation().getDistance(local3d.getTranslation()) / local2d.getTranslation().getNorm()) * 100;

            print("tX: " + tX + ", tY: " + tY);
            print("2d:  " + local2d);
            print("3d: " + local3d);
            print("Percent Difference: " + Math.round(percentdiffernce * 100.0) / 100.0 + "%");
            print("-----------------------------------------------------------");
        }
    }

    /* 2d */
    public Translation2d getDistance(Rotation2d ty){
        double heightDifference = Constants.Vision.goalHeight - Constants.Vision.limelightHeight;
        Rotation2d combinedAngle = Constants.Vision.limelightAngle.plus(ty);

        return new Translation2d((heightDifference / combinedAngle.getTan()), 0);
    }

    /* 2d */
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

    /* 3d */
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

        return new Pose2d(fieldToRobotTranslation, robotHeading);
    }
    
    public void print(Object print){
        System.out.println(print);
    }
}