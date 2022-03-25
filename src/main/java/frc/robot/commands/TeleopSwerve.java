package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private double tRate = Double.MAX_VALUE;
    SlewRateLimiter translationFilter = new SlewRateLimiter(tRate);
    private double xRate = Constants.Swerve.slewRateLimiterAmount;
    SlewRateLimiter xAxisFilter = new SlewRateLimiter(xRate);
    private double yRate = Constants.Swerve.slewRateLimiterAmount;
    SlewRateLimiter yAxisFilter = new SlewRateLimiter(yRate);
    private double rRate = Constants.Swerve.slewRateLimiterAmount;
    SlewRateLimiter rAxisFilter = new SlewRateLimiter(rRate);


    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.openLoop = openLoop;
    }

    public Translation2d getTranslation2d() {

        double yAxis = -controller.getRawAxis(translationAxis);
        double yAxisAbs = Math.abs(yAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double xAxisAbs = Math.abs(xAxis);

        /* Deadbands */
        yAxis = (yAxisAbs < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (xAxisAbs < Constants.stickDeadband) ? 0 : xAxis;

        /* Squaring Inputs */
        // yAxis *= yAxis;
        // xAxis *= xAxis;
        yAxis *= yAxisAbs;
        xAxis *= xAxisAbs;

        // double combinedAxis = xAxis + yAxis;
        // double filteredCombinedAxis = translationFilter.calculate(combinedAxis);
        // double filteredIsolatedX = filteredCombinedAxis * (xAxis / combinedAxis);
        // double filteredIsolatedY = filteredCombinedAxis * (yAxis / combinedAxis);

        return new Translation2d(
            //filteredIsolatedX * Math.signum(xAxis),
            yAxisFilter.calculate(yAxis) * s_Swerve.gethighLowGear(), 
            //filteredIsolatedY * Math.signum(yAxis)
            xAxisFilter.calculate(xAxis) * s_Swerve.gethighLowGear()
        ).times(Constants.Swerve.maxSpeed);
    }

    @Override
    public void execute() {

        if(States.shooterState != ShooterStates.preShoot){
        
        //double yAxis = -controller.getRawAxis(translationAxis);
        //double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        double rAxisAbs = Math.abs(rAxis);
        
        /* Deadbands */
        //yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        //xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (rAxisAbs < Constants.stickDeadband) ? 0 : rAxis;

        /* Squaring Inputs */ 
        //yAxis *= Math.abs(yAxis);
        //xAxis *= Math.abs(xAxis);
        rAxis *= rAxisAbs;

        if(Constants.tuningMode) {
            double r = Swerve.turnRateLimiting.getDouble(0);
            double t = Swerve.transRateLimiting.getDouble(0);
            //double y = Swerve.rateLimiting.getDouble(0), x = Swerve.rateLimiting.getDouble(0), r = Swerve.rateLimiting.getDouble(0);
            if( r != rRate || t != tRate) { //y != yRate || x != xRate ||
                //yRate = y;
                //xRate = x;
                rRate = r;
                tRate = t;
                //yAxisFilter = new SlewRateLimiter(y);
                //xAxisFilter = new SlewRateLimiter(x);
                rAxisFilter = new SlewRateLimiter(r);
                translationFilter = new SlewRateLimiter(t);
            }
        }

        translation = getTranslation2d();//new Translation2d(yAxisFilter.calculate(yAxis) * s_Swerve.gethighLowGear(), xAxisFilter.calculate(xAxis) * s_Swerve.gethighLowGear()).times(Constants.Swerve.maxSpeed);
        rotation = /*rAxisFilter.calculate(*/rAxis/*)*/ * Constants.Swerve.maxAngularVelocity * s_Swerve.gethighLowGear();
        s_Swerve.drive(translation, rotation, openLoop);
        }
    }

}
