package frc.Controllers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkMax;
import frc.lib.math.PIDGains;

/**
 * Thin Spark Max wrapper to make setup easier, 
 * and automatically initalize the internal CANEncoder and PIDController.
 */
public class LazySparkMAX extends CANSparkMax {
    protected SparkMaxPIDController m_pidController;
    protected RelativeEncoder m_encoder;

    /**
     * Config a Spark Max using sparkConstants.
     * 
     * @param intakemotorconstants
     */
    public LazySparkMAX(SparkConstants motorConstants) {
        super(motorConstants.deviceId, motorConstants.motorType);
        super.restoreFactoryDefaults();
        super.setSmartCurrentLimit(motorConstants.smartCurrentLimit);
        super.enableVoltageCompensation(12);
        super.setIdleMode(motorConstants.idleMode);
        super.setInverted(motorConstants.inverted);
        super.burnFlash();

        m_pidController = super.getPIDController();
        m_encoder = super.getEncoder();

        m_encoder.setPosition(0);
    }

    
    public void set(ControlType kdutycycle, double setpoint) {
        m_pidController.setReference(setpoint, kdutycycle);
    }
    
    /**
     * Config PID Gains and Peak Outputs using PIDGains
     * @param pidGains
     */
    public void configPID(PIDGains pidGains){
        m_pidController.setP(pidGains.kP);
        m_pidController.setI(pidGains.kI);
        m_pidController.setD(pidGains.kD);
        m_pidController.setFF(pidGains.kFF);
        m_pidController.setOutputRange(pidGains.kMaxReverse, pidGains.kMaxForward);
    }

    /**
     * Returns PIDGains and configured Peak Outputs from internal PIDController
     * @return
     */
    public PIDGains getPIDGains(){
        PIDGains pidGains = new PIDGains(
            m_pidController.getP(), 
            m_pidController.getI(), 
            m_pidController.getD(), 
            m_pidController.getFF(), 
            m_pidController.getOutputMax(), 
            m_pidController.getOutputMin()
        );
        return pidGains;
    }

    /**
     * Set the internal pid controllers feedback device.
     * @param sensor
     */
    public void setFeedbackDevice(MotorFeedbackSensor sensor) {
        m_pidController.setFeedbackDevice(sensor);
    }

    public double getPosition(){
        return m_encoder.getPosition();
    }

    public void setPosition(double position){
        m_encoder.setPosition(position);
    }

    public double getVelocity(){
        return m_encoder.getVelocity();
    }

    /**
     * Set conversion factor of encoder, so that position is in meters and velocity is in MPS.
     * @param circumference Circumference of wheel in meters
     * @param gearRatio Reduction to wheel. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorMeters(double circumference, double gearRatio){
        m_encoder.setPositionConversionFactor(circumference / gearRatio);
        m_encoder.setVelocityConversionFactor((circumference / gearRatio) / 60.0);
    }

    /**
     * Set conversion factor of encoder, so that position is in rotations and velocity is in RPM
     * @param gearRatio Reduction to mech. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorRotations(double gearRatio){
        m_encoder.setPositionConversionFactor(1 / gearRatio);
        m_encoder.setVelocityConversionFactor(1 / gearRatio);
    }

    /**
     * Set conversion factor of encoder, so that position is in degrees and velocity is in degrees/min
     * @param gearRatio Reduction to mech. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorDegrees(double gearRatio){
        m_encoder.setPositionConversionFactor(360 / gearRatio);
        m_encoder.setVelocityConversionFactor(360 / gearRatio);
    }

    public void setStatusFrames(int period){
        period = period > 65535 ? 65535 : period;
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, period);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, period);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, period);
    }



    
}