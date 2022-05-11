package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.LazyTalonFX;
import frc.lib.math.Conversions;
import frc.lib.newWpilibUtils.InterpolatingTreeMap;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;

public class Shooter extends SubsystemBase {
    private LazyTalonFX shooterMotorParent;
    private LazyTalonFX shooterMotorChild;
    private Limelight limelight;

    private InterpolatingTreeMap<Double, Double> shooterMap = new InterpolatingTreeMap<>();

    private ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
    private NetworkTableEntry shootP = tuning.add("Shoot P", 0).getEntry();
    private NetworkTableEntry shootI = tuning.add("Shoot I", 0).getEntry();
    private NetworkTableEntry shootD = tuning.add("Shoot D", 0).getEntry();
    private NetworkTableEntry shootRPM = tuning.add("Shoot RPM", 0.).getEntry();
    private NetworkTableEntry setShootRPM = tuning.add("Set Shoot RPM", 0.).getEntry();
    private NetworkTableEntry tuneShoot = tuning.add("Tune Shoot", false).getEntry();

    private ShuffleboardTab testing = Shuffleboard.getTab("Testing");
    private ShuffleboardTab drivers = Shuffleboard.getTab("Drivers");
    private NetworkTableEntry getDistEntry = drivers.add("GetDist", 0).getEntry();

    private double tuningShooterKP;
    private double tuningShooterKI;
    private double tuningShooterKD;

    public Shooter(Vision m_Vision) {
        shooterMotorParent = new LazyTalonFX(Constants.Shooter.parentShooterConstants);
        shooterMotorChild = new LazyTalonFX(Constants.Shooter.childShooterConstants);
        shooterMotorChild.follow(shooterMotorParent);
        limelight = m_Vision.getLimelight();

        tuningShooterKP = Constants.Shooter.shootKP;
        tuningShooterKI = Constants.Shooter.shootKI;
        tuningShooterKD = Constants.Shooter.shootKD;

        testing.add("Start Shooter Motors", new InstantCommand(
            () -> setPower(0.5)
        ));

        testing.add("Stop Shooter Motors", new InstantCommand(
            () -> setPower(0)
        ));

        for (double[] map : Constants.Shooter.shooterMap) {
            shooterMap.put(map[0], map[1]);
        }
    }

    public double getShooterRPM(){
        return Conversions.falconToRPM(shooterMotorParent.getSelectedSensorVelocity(), Constants.Shooter.shooterGearRatio);
    }

    public double getTargetRPM(){
        return shooterMap.get(limelight.getDistance().plus(new Translation2d(Constants.Vision.goalDiameter/2, 0)).getNorm());
    }

    public void setShooterRPM(double shooterRPM){
        double falconVelocity = Conversions.RPMToFalcon(shooterRPM, Constants.Shooter.shooterGearRatio);
        shooterMotorParent.set(ControlMode.Velocity, falconVelocity);
    }

    public boolean isShooterReady() {
        if(getShooterRPM() >= getTargetRPM() - Constants.Shooter.tolerance && getShooterRPM() <= getTargetRPM() + Constants.Shooter.tolerance) {
            return true;
        }
        return false;
    }

    public void setPower(double power){
        shooterMotorParent.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
        getDistEntry.setDouble(getTargetRPM());

        switch(States.shooterState){
            case disabled:
                shooterMotorParent.set(ControlMode.PercentOutput, 0);
                break;
                
            case preShoot:
                if (Constants.Shooter.calibrationMode){
                    setShooterRPM(setShootRPM.getDouble(0));
                } else{
                    setShooterRPM(getTargetRPM());
                }
                break;

            case lowPreShoot:
                if(Constants.Shooter.calibrationMode) {
                    setShooterRPM(setShootRPM.getDouble(0));
                } else {
                    setShooterRPM(Constants.Shooter.shooterLowMap[1]);
                }
                break;

            default:
                break;
        }

        if(Constants.tuningMode || Constants.Shooter.calibrationMode) {
            shootRPM.setDouble(this.getShooterRPM());
        }

        if(Constants.tuningMode) {
            double p, i, d;
            if(tuneShoot.getBoolean(false)) {
                p = shootP.getDouble(0); i = shootI.getDouble(0); d = shootD.getDouble(0);
                if(p != tuningShooterKP) {
                    tuningShooterKP = p;
                    shooterMotorParent.config_kP(0, tuningShooterKP);
                }

                if(i != tuningShooterKI) {
                    tuningShooterKI = i;
                    shooterMotorParent.config_kI(0, tuningShooterKI);
                }

                if(d != tuningShooterKD) {
                    tuningShooterKD = d;
                    shooterMotorParent.config_kD(0, tuningShooterKD);
                }
                
                setShooterRPM(setShootRPM.getDouble(0));
            }
        }
    }
}
