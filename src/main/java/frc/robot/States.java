package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class States {

    private static final ShuffleboardTab driver = Shuffleboard.getTab(Constants.driverTab);
    private static final NetworkTableEntry preshoot = driver.add("Preshoot", false).getEntry();
    private static final NetworkTableEntry preshootMode = driver.add("Preshoot Mode", ShooterStates.preShoot.label).getEntry();
    private static final NetworkTableEntry canClimb = driver.add("Can Climb", false).getEntry();

    /**
     * Shooter States:
     * </p> disabled: Auto Aim Disabled, Shooter Disabled, and Shooter Tilt at 0
     * </p> preShoot: Drivetrain Auto Aim, Shooter SpinUp, Shooter Tilt to angle
     */
    public static enum ShooterStates {
        disabled("disabled"), preShoot("high goal"), lowPreShoot("low goal");

        public final String label;
        private ShooterStates(String label) {
            this.label = label;
        }
    }

    public static enum IntakeStates {
        disabled, intaking, outtaking, feeding, reverseFeeding, intakeAndFeed
    }

    public static enum IntakeExtendStates {
        disabled, deployIntake, retractIntake
    }

    public static enum ClimberStates {
        disabled, extendClimberPiston, retractClimberPiston, extendClimber, retractClimber
    }

    public static ShooterStates shooterState = ShooterStates.disabled;
    public static IntakeStates intakeState = IntakeStates.disabled;
    public static IntakeExtendStates intakeExtendState = IntakeExtendStates.disabled;
    public static ClimberStates climberState = ClimberStates.disabled;
    public static boolean climbAllowed = false;
    private static ShooterStates activeShooterMode = ShooterStates.preShoot;
    
    public static void reset() {
        shooterState = ShooterStates.disabled;
        intakeState = IntakeStates.disabled;
        intakeExtendState = IntakeExtendStates.disabled;
        climberState = ClimberStates.disabled;
        climbAllowed = false;
    }

    public static void toggleIntake() {
        switch(States.intakeExtendState) {
            case deployIntake:
                retractIntake();
                break;
            case retractIntake:
                deployIntake();
                break;
            case disabled:
                deployIntake();
                break;
        }
    }

    public static void deployIntake(){
        States.intakeExtendState = IntakeExtendStates.deployIntake;
    }

    public static void retractIntake(){
        States.intakeExtendState = IntakeExtendStates.retractIntake;
    }

    public static void feed() {
        States.intakeState = IntakeStates.feeding;
    }

    public static void intakeAndFeed() {
        States.intakeState = States.IntakeStates.intakeAndFeed;
    }

    public static void stopIntake() {
        States.intakeState = IntakeStates.disabled;
    }

    public static void setActiveShooterMode(ShooterStates state) {
        preshootMode.setString(state.label);
        activeShooterMode = state;
    }

    public static void activateShooter() {
        preshoot.setBoolean(true);
        States.shooterState = activeShooterMode;//ShooterStates.preShoot;
    }

    public static void deactivateShooter() {
        preshoot.setBoolean(false);
        States.shooterState = ShooterStates.disabled;
    }

    public static void intake() {
        if(States.intakeState != IntakeStates.feeding)
            States.intakeState = IntakeStates.intaking;
            States.intakeExtendState = IntakeExtendStates.deployIntake;
    }

    public static void outtake() {
        //States.intakeState = IntakeStates.outtaking;
        States.intakeState = IntakeStates.reverseFeeding;
        States.intakeExtendState = IntakeExtendStates.deployIntake;
    }

    public static void stopClimber() {
        States.climberState = ClimberStates.disabled;
    }

    public static void allowClimb() {
        canClimb.setBoolean(true);
        climbAllowed = true;
    }

    public static void disallowClimb() {
        canClimb.setBoolean(false);
        climbAllowed = false;
    }

    public static void extendClimberPiston() {
        deployIntake();
        States.climberState = ClimberStates.extendClimberPiston;
    }

    public static void retractClimberPiston() {
        deployIntake();
        States.climberState = ClimberStates.retractClimberPiston;
    }

    public static void extendClimber() {
        deployIntake();
        States.climberState = ClimberStates.extendClimber;
    }

    public static void retractClimber() {
        deployIntake();
        States.climberState = ClimberStates.retractClimber;
    }

}
