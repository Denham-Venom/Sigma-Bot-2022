package frc.lib.Controllers;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.lib.math.PIDGains;

/**
 * Thin Falcon wrapper to make setup easier.
 */
public class LazyTalonFX extends TalonFX {

    /**
     * Config using individual parameters.
     * @param deviceNumber
     * @param allConfigs
     * @param neutralMode
     * @param invertType
     * @param slowStatusFrame
     */
    public LazyTalonFX(int deviceNumber, TalonFXConfiguration allConfigs, NeutralMode neutralMode, InvertType invertType, boolean slowStatusFrame){
        super(deviceNumber);
        super.configFactoryDefault();
        super.configAllSettings(allConfigs);
        super.setNeutralMode(neutralMode);
        super.setInverted(invertType);
        super.setSelectedSensorPosition(0);

        if (slowStatusFrame){
            super.setStatusFramePeriod(StatusFrame.Status_1_General, 255, 30);
            super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, 30);
        }
    }

    /**
     * Config using talonFxConstants.
     * @param talonFxConstants
     */
    public LazyTalonFX(TalonFxConstants talonFxConstants){
        super(talonFxConstants.deviceNumber);
        super.configFactoryDefault();
        super.configAllSettings(talonFxConstants.allConfigs);
        super.setNeutralMode(talonFxConstants.neutralMode);
        super.setInverted(talonFxConstants.invertType);
        super.setSelectedSensorPosition(0);

        if (talonFxConstants.slowStatusFrame){
            super.setStatusFramePeriod(StatusFrame.Status_1_General, 255, 30);
            super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, 30);
        }
    }
    
    /**
     * Config PID Gains and Peak Outputs using PIDGains
     * @param pidGains
     */
    public void configPID(PIDGains pidGains){
        super.config_kP(0, pidGains.kP);
        super.config_kI(0, pidGains.kI);
        super.config_kD(0, pidGains.kD);
        super.config_kF(0, pidGains.kFF);
        super.configPeakOutputForward(pidGains.kMaxForward);
        super.configPeakOutputReverse(pidGains.kMaxReverse);
    }    
}