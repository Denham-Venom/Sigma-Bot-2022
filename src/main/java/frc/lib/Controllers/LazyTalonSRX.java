package frc.lib.Controllers;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/**
 * Thin Talon SRX wrapper to make setup easier.
 */
public class LazyTalonSRX extends TalonSRX {

    /**
     * Config using individual parameters.
     * @param deviceNumber
     * @param allConfigs
     * @param neutralMode
     * @param invertType
     * @param slowStatusFrame
     */
    public LazyTalonSRX(int deviceNumber, TalonSRXConfiguration allConfigs, NeutralMode neutralMode, InvertType invertType, boolean slowStatusFrame){
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
     * Config using talonSRXConstants.
     * @param talonSRXConstants
     */
    public LazyTalonSRX(TalonSRXConstants talonSRXConstants){
        super(talonSRXConstants.deviceNumber);
        super.configFactoryDefault();
        super.configAllSettings(talonSRXConstants.allConfigs);
        super.setNeutralMode(talonSRXConstants.neutralMode);
        super.setInverted(talonSRXConstants.invertType);
        super.setSelectedSensorPosition(0);

        if (talonSRXConstants.slowStatusFrame){
            super.setStatusFramePeriod(StatusFrame.Status_1_General, 255, 30);
            super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, 30);
        }
    }
}