package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.LazyTalonSRX;
import frc.lib.newWpilibUtils.InterpolatingTreeMap;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;

public class Hood extends SubsystemBase {
    private LazyTalonSRX hoodMotor;
    private Limelight limelight;
    
    private InterpolatingTreeMap<Double, Double> hoodMap = new InterpolatingTreeMap<>();
    private PIDController hoodController;    
    private DigitalInput hoodLimit;
    private Encoder hoodEncoder;

    private boolean homingDone = false;

    private ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
    private NetworkTableEntry hoodP = tuning.add("Hood P", 0).getEntry();
    private NetworkTableEntry hoodI = tuning.add("Hood I", 0).getEntry();
    private NetworkTableEntry hoodD = tuning.add("Hood D", 0).getEntry();
    private NetworkTableEntry hoodAng = tuning.add("Hood Angle", 0.).getEntry();
    private NetworkTableEntry setHoodAng = tuning.add("Set Hood Angle", 0.).getEntry();
    private NetworkTableEntry tuneHood = tuning.add("Tune Hood", false).getEntry();
    private NetworkTableEntry hoodPIDOut = tuning.add("HoodPIDOut", 0.).getEntry();
    private NetworkTableEntry hoodLimitSwitchPressed = tuning.add("HoodLimitPressed", false).getEntry();

    private ShuffleboardTab testing = Shuffleboard.getTab("Testing");
    private ShuffleboardTab drivers = Shuffleboard.getTab("Drivers");
    private NetworkTableEntry hoodReady = drivers.add("Hood Ready", false).getEntry();    

    public Hood(Vision m_Vision) {
        hoodMotor = new LazyTalonSRX(Constants.Hood.hoodConstants);
        limelight = m_Vision.getLimelight();

        hoodController = new PIDController(
            Constants.Hood.hoodKP, 
            Constants.Hood.hoodKI, 
            Constants.Hood.hoodKD
        );

        hoodEncoder = new Encoder(1, 2, false, Encoder.EncodingType.k4X);
        hoodEncoder.reset();
        hoodEncoder.setDistancePerPulse(360. * Constants.Hood.hoodGearRatio / 2048.); //degrees

        hoodLimit = new DigitalInput(Constants.Hood.hoodLimitSwitchID);

        testing.add("UP Hood Motor", new InstantCommand(
            () -> hoodMotor.set (ControlMode.PercentOutput, 0.15)
        ));

        testing.add("DOWN Hood Motor", new InstantCommand(
            () -> hoodMotor.set (ControlMode.PercentOutput, -0.15)
        ));

        testing.add("STOP Hood Motor", new InstantCommand(
            () -> hoodMotor.set (ControlMode.PercentOutput, 0)
        ));   

        for (double[] map : Constants.Shooter.shooterMap) {
            hoodMap.put(map[0], map[2]);
        }
    }

    public void resetHood() {
        homingDone = false;
    }

    public double getHoodAngle(){
        return hoodEncoder.getDistance() + Constants.Hood.hoodAngleOffset;
    }

    public double getTargetAngle(){
        return hoodMap.get(limelight.getDistance().plus(new Translation2d(Constants.Vision.goalDiameter/2, 0)).getNorm());
    }

    public void setHoodAngle(double targetAngle){
        if (targetAngle > Constants.Hood.hoodHighLimit){
            targetAngle = Constants.Hood.hoodHighLimit;
        }
        else if (targetAngle < Constants.Hood.hoodLowLimit){
            targetAngle = Constants.Hood.hoodLowLimit;
        }
        
        double output = hoodController.calculate(getHoodAngle(), targetAngle);
        if(output < 0) {
            output = getHoodLimitSwitch() ? 0 : (output + Constants.Hood.hoodDownFF);
        } else if(output > 0) {
            output += Constants.Hood.hoodKF;
        }

        hoodReady.setBoolean(Math.abs(targetAngle - getHoodAngle()) < Constants.Hood.hoodControllerToleranceDegrees);
        hoodPIDOut.setDouble(output);
        hoodMotor.set(ControlMode.PercentOutput, output);
    }

    public void setHoodPower(double scale) {
        hoodMotor.set(ControlMode.PercentOutput, scale * 0.2);
    }

    public void stopHood() {
        hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean getHoodLimitSwitch() {
        return !hoodLimit.get();
    }

    @Override
    public void periodic() {
        hoodLimitSwitchPressed.setBoolean(getHoodLimitSwitch());
        if(getHoodLimitSwitch()){
            hoodEncoder.reset();
        }

        if(!homingDone) {
            if(getHoodLimitSwitch()){
                homingDone = true;
            }
            setHoodPower(-1);
            return;
        }

        switch(States.shooterState){
            case disabled:
                if(!Constants.tuningMode) {
                    setHoodAngle(10);
                }
                break;
                
            case preShoot:
                if (Constants.Shooter.calibrationMode){
                    setHoodAngle(setHoodAng.getDouble(0));
                } else{
                    setHoodAngle(hoodMap.get(getTargetAngle()));
                }
                break;

            case lowPreShoot:
                if(Constants.Shooter.calibrationMode) {
                    setHoodAngle(setHoodAng.getDouble(0));
                } else {
                    setHoodAngle(Constants.Shooter.shooterLowMap[2]);
                }
                break;

            default:
                break;
        }
        
        if(Constants.tuningMode || Constants.Shooter.calibrationMode) {
            hoodAng.setDouble(this.getHoodAngle());
        }
        
        if(Constants.tuningMode) {
            double p, i, d;
            if(tuneHood.getBoolean(false)) {
                p = hoodP.getDouble(0); i = hoodI.getDouble(0); d = hoodD.getDouble(0);
                if(p != hoodController.getP() || i != hoodController.getI() || d != hoodController.getD()) {
                    hoodController.setPID(p, i, d);
                }
                setHoodAngle(setHoodAng.getDouble(0));
            }
        }
    }
}
