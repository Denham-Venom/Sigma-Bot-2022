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
import frc.lib.util.Interpolatable;
import frc.lib.util.InterpolatableTreeMap;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;

public class Hood extends SubsystemBase {
    private LazyTalonSRX hoodMotor;
    private Limelight limelight;
    
    private InterpolatableTreeMap<Double> hoodMap = new InterpolatableTreeMap<>();
    private PIDController hoodController;

    private boolean homingDone = false;
    private DigitalInput hoodLimit = new DigitalInput(Constants.Shooter.hoodLimitSwitchID);
    private Encoder hoodEncoder = new Encoder(1, 2, false, Encoder.EncodingType.k4X);

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
        hoodMotor = new LazyTalonSRX(Constants.Shooter.hoodConstants);
        limelight = m_Vision.getLimelight();

        hoodController = new PIDController(
            Constants.Shooter.hoodKP, 
            Constants.Shooter.hoodKI, 
            Constants.Shooter.hoodKD
        );
        hoodController.setTolerance(Constants.Shooter.hoodControllerToleranceDegrees, 0.5);
        hoodEncoder.reset();
        hoodEncoder.setDistancePerPulse(360. * Constants.Shooter.hoodGearRatio / 2048.); //degrees

        testing.add("UP Hood Motor", new InstantCommand(
            () -> hoodMotor.set (ControlMode.PercentOutput, 0.15)
        ));

        testing.add("DOWN Hood Motor", new InstantCommand(
            () -> hoodMotor.set (ControlMode.PercentOutput, -0.15)
        ));

        testing.add("STOP Hood Motor", new InstantCommand(
            () -> hoodMotor.set (ControlMode.PercentOutput, 0)
        ));

        for (int i = 0; i < Constants.Shooter.shooterMap.length; ++i) {
            hoodMap.set(Constants.Shooter.shooterMap[i][0], Interpolatable.interDouble(Constants.Shooter.shooterMap[i][2]));
        }
        
    }

    public void resetHood() {
        homingDone = false;
    }

    public double getHoodAngle(){
        return hoodEncoder.getDistance() + Constants.Shooter.hoodAngleOffset;
    }

    public double getTargetAngle(){
        return hoodMap.get(limelight.getDistance().plus(new Translation2d(Constants.Vision.goalDiameter/2, 0)).getNorm());
    }

    public void setHoodAngle(double hoodAngle){
        if (hoodAngle > Constants.Shooter.hoodHighLimit){
            hoodAngle = Constants.Shooter.hoodHighLimit;
        }
        else if (hoodAngle < Constants.Shooter.hoodLowLimit){
            hoodAngle = Constants.Shooter.hoodLowLimit;
        }
        hoodController.setSetpoint(hoodAngle);
        if(hoodController.atSetpoint()) {
            hoodMotor.set(ControlMode.PercentOutput, 0);
            hoodReady.setBoolean(true);
        return;
        }
        hoodReady.setBoolean(false);
        
        double output = hoodController.calculate(getHoodAngle());// + Constants.Shooter.hoodPID.kFF;
        if(output < 0) {
            if(!hoodLimit.get()) output = 0;
            else output += Constants.Shooter.hoodDownFF;
        } else if(output > 0) {
            output += Constants.Shooter.hoodKF;
        }
        hoodPIDOut.setDouble(output);
        hoodMotor.set(ControlMode.PercentOutput, output);//Conversions.degreesToFalcon(hoodAngle, Constants.Shooter.hoodGearRatio));
    }

    public void setHoodPower(double scale) {
        hoodMotor.set(ControlMode.PercentOutput, scale * 0.2);//SmartDashboard.getNumber("HoodTestPow", 0));
    }

    public void stopHood() {
        hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {    
        boolean hoodDown = !hoodLimit.get();
        hoodLimitSwitchPressed.setBoolean(hoodDown);
        if(hoodDown) hoodEncoder.reset();

        if(!homingDone) {
            if(hoodDown) homingDone = true;
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
