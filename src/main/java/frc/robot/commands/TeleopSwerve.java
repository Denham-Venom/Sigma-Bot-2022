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
    private double tRate = Constants.Swerve.transRateLimit;
    SlewRateLimiter translationFilter = new SlewRateLimiter(tRate);
    private double rRate = Constants.Swerve.rotRateLimit;
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
        double speedMultiplier = s_Swerve.gethighLowGear();

        double yAxis = -controller.getRawAxis(translationAxis);
        double yAxisAbs = Math.abs(yAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double xAxisAbs = Math.abs(xAxis);

        /* Deadbands */
        yAxis = (yAxisAbs < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (xAxisAbs < Constants.stickDeadband) ? 0 : xAxis;

        /* Squaring Inputs */
        double squaredX = xAxis * xAxis;
        double squaredY = yAxis * yAxis;

        double combinedAxis = squaredX + squaredY;
        double filteredCombinedAxis = translationFilter.calculate(combinedAxis);
        double filteredIsolatedX = filteredCombinedAxis * (squaredX / combinedAxis);
        double filteredIsolatedY = filteredCombinedAxis * (squaredY / combinedAxis);

        return new Translation2d(
            filteredIsolatedX * Math.signum(xAxis) * speedMultiplier,
            filteredIsolatedY * Math.signum(yAxis) * speedMultiplier
        ).times(Constants.Swerve.maxSpeed);
    }

    @Override
    public void execute() {

        if(States.shooterState != ShooterStates.preShoot){

            double rAxis = -controller.getRawAxis(rotationAxis);
            double rAxisAbs = Math.abs(rAxis);
            
            /* Deadbands */
            rAxis = (rAxisAbs < Constants.stickDeadband) ? 0 : rAxis;

            /* Squaring Inputs */ 
            rAxis *= rAxisAbs;

            translation = getTranslation2d();//new Translation2d(yAxisFilter.calculate(yAxis) * s_Swerve.gethighLowGear(), xAxisFilter.calculate(xAxis) * s_Swerve.gethighLowGear()).times(Constants.Swerve.maxSpeed);
            rotation = rAxisFilter.calculate(rAxis) * Constants.Swerve.maxAngularVelocity * s_Swerve.gethighLowGear();
            s_Swerve.drive(translation, rotation, openLoop);

            if(Constants.tuningMode) {
                double r = Swerve.turnRateLimiting.getDouble(0), t = Swerve.transRateLimiting.getDouble(0);
                if(r != rRate || t != tRate) {
                    rRate = r;
                    tRate = t;
                    rAxisFilter = new SlewRateLimiter(r);
                    translationFilter = new SlewRateLimiter(t);
                }
            }
        }
    }

}
