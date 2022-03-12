package frc.Controllers;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkConstants {
    public final int deviceId;
    public final MotorType motorType;
    public final int smartCurrentLimit;
    public final IdleMode idleMode;
    public final boolean inverted;
    public final boolean useAlternateEncoder;
    public final int encoderCountsPerRev;

    public SparkConstants(int deviceId, MotorType motorType, int smartCurrentLimit, IdleMode idleMode, boolean inverted, boolean useAlternateEncoder, int encoderCountsPerRev) {
        this.deviceId = deviceId;
        this.motorType = motorType;
        this.smartCurrentLimit = smartCurrentLimit;
        this.idleMode = idleMode;
        this.inverted = inverted;
        this.useAlternateEncoder = useAlternateEncoder;
        this.encoderCountsPerRev = encoderCountsPerRev;
    }
}