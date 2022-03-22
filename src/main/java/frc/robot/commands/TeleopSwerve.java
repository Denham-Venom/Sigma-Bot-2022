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
    SlewRateLimiter xAxisFilter = new SlewRateLimiter(Constants.Swerve.slewRateLimiterAmount);
    SlewRateLimiter yAxisFilter = new SlewRateLimiter(Constants.Swerve.slewRateLimiterAmount);
    SlewRateLimiter rAxisFilter = new SlewRateLimiter(Constants.Swerve.slewRateLimiterAmount);


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
        double xAxis = -controller.getRawAxis(strafeAxis);

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;

        /* Squaring Inputs */
        yAxis *= yAxis;
        xAxis *= xAxis;

        return new Translation2d(yAxisFilter.calculate(yAxis) * s_Swerve.gethighLowGear(), xAxisFilter.calculate(xAxis) * s_Swerve.gethighLowGear()).times(Constants.Swerve.maxSpeed);
    }

    @Override
    public void execute() {

        if(States.shooterState != ShooterStates.preShoot){
        
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);       
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        /* Squaring Inputs */ 
        yAxis *= yAxis;
        xAxis *= xAxis;
        rAxis *= rAxis;

        if(Constants.tuningMode) {
            yAxisFilter = new SlewRateLimiter(Swerve.rateLimiting.getDouble(0));
            xAxisFilter = new SlewRateLimiter(Swerve.rateLimiting.getDouble(0));
            rAxisFilter = new SlewRateLimiter(Swerve.rateLimiting.getDouble(0));
        }

        translation = new Translation2d(yAxisFilter.calculate(yAxis) * s_Swerve.gethighLowGear(), xAxisFilter.calculate(xAxis) * s_Swerve.gethighLowGear()).times(Constants.Swerve.maxSpeed);
        rotation = rAxisFilter.calculate(rAxis) * Constants.Swerve.maxAngularVelocity * s_Swerve.gethighLowGear();
        s_Swerve.drive(translation, rotation, openLoop);
        }
    }

}
