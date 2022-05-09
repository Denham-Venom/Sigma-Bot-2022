package frc.robot.commands;

import frc.lib.math.Boundaries;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSupplier;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier rotationSupplier;
    private boolean openLoop;

    /* Driver control */
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        /* Deadbands */
        double yAxis = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.stickDeadband);
        double xAxis = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.stickDeadband);
        double rAxis = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.stickDeadband);

        /* Squaring Inputs */
        yAxis = Boundaries.squareInput(yAxis);
        xAxis = Boundaries.squareInput(xAxis);
        rAxis = Boundaries.squareInput(rAxis);

        Translation2d translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed).times(s_Swerve.gethighLowGear());
        double rotation = rAxis * Constants.Swerve.maxAngularVelocity * s_Swerve.gethighLowGear();

        if(States.shooterState != ShooterStates.preShoot){        
            s_Swerve.drive(translation, rotation, openLoop);
        }
    }
}
