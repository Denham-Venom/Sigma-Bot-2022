package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.lib.util.Limelight.ledStates;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;


public class Vision extends SubsystemBase {
    private PoseEstimator m_poseEstimator;
    private Limelight limelight;
    
    public Vision(PoseEstimator m_poseEstimator) {
        this.m_poseEstimator = m_poseEstimator;

        limelight = new Limelight(
            Constants.Vision.limelightHeight,
            Constants.Vision.limelightAngle,
            Constants.Vision.goalHeight
        );
        
        limelight.ledState(ledStates.on);
    }

    public Limelight getLimelight() {
        return this.limelight;
    }

    @Override
    public void periodic() {
        if (States.shooterState == ShooterStates.preShoot){
            limelight.ledState(ledStates.on);
        }
        else{
            limelight.ledState(ledStates.on);
        }

        if(Constants.Shooter.calibrationMode){
            limelight.ledState(ledStates.on);
        }

        if(limelight.hasTarget() && m_poseEstimator.readyToUpdateVision()){
            m_poseEstimator.updateVision(limelight.getDistance(), limelight.getTx(), limelight.getLatency());
        }
    }
}
