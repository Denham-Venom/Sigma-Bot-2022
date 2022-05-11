package frc.lib.Controllers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.CANSparkMax;

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

    public void setStatusFrames(int period){
        period = period > 65535 ? 65535 : period;
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, period);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, period);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, period);
    }



    
}