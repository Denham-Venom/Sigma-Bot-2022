package frc.robot;

public class States {

    /**
     * Shooter States:
     * </p> disabled: Auto Aim Disabled, Shooter Disabled, and Shooter Tilt at 0
     * </p> preShoot: Drivetrain Auto Aim, Shooter SpinUp, Shooter Tilt to angle
     */
    public static enum ShooterStates {
        disabled, preShoot
    }

    public static enum IntakeStates {
        disabled, intaking, outtaking, feeding
    }

    public static enum IntakeExtendStates {
        disabled, deployIntake, retractIntake
    }

    public static ShooterStates shooterState = ShooterStates.disabled;
    public static IntakeStates intakeState = IntakeStates.disabled;
    public static IntakeExtendStates intakeExtendState = IntakeExtendStates.disabled;

    static void deployIntake(){
        States.intakeExtendState = IntakeExtendStates.deployIntake;
    }

    static void retractIntake(){
        States.intakeExtendState = IntakeExtendStates.retractIntake;
    }

    static void feed() {
        States.intakeState = IntakeStates.feeding;
    }

    static void stopIntake() {
        States.intakeState = IntakeStates.disabled;
    }

    static void activateShooter() {
        States.shooterState = ShooterStates.preShoot;
    }

    static void deactivateShooter() {
        States.shooterState = ShooterStates.disabled;
    }

    static void intake() {
        if(States.intakeState != IntakeStates.feeding)
            States.intakeState = IntakeStates.intaking;
    }

    static void outtake() {
        States.intakeState = IntakeStates.outtaking;
    }

}
