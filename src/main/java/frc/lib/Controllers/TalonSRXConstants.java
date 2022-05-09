package frc.lib.Controllers;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class TalonSRXConstants {
    public final int deviceNumber;
    public final TalonSRXConfiguration allConfigs;
    public final NeutralMode neutralMode;
    public final InvertType invertType;
    public final boolean slowStatusFrame;    
    
    /**
     * Constants to be used with LazyTalonSRX Util
     * @param deviceNumber
     * @param allConfigs
     * @param neutralMode
     * @param invertType
     * @param slowStatusFrames
     */
    public TalonSRXConstants(int deviceNumber, TalonSRXConfiguration allConfigs, NeutralMode neutralMode, InvertType invertType, boolean slowStatusFrame) {
        this.deviceNumber = deviceNumber;
        this.allConfigs = allConfigs;
        this.neutralMode = neutralMode;
        this.invertType = invertType;
        this.slowStatusFrame = slowStatusFrame;
    }
}
