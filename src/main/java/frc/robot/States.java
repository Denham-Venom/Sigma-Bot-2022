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

    public static enum ClimberStates {
        disabled, extendClimberPiston, retractClimberPiston, extendClimber, retractClimber, fullClimb
    }

    public static ShooterStates shooterState = ShooterStates.disabled;
    public static IntakeStates intakeState = IntakeStates.disabled;
    public static IntakeExtendStates intakeExtendState = IntakeExtendStates.disabled;
    public static ClimberStates climberState = ClimberStates.disabled;

    public static void deployIntake(){
        States.intakeExtendState = IntakeExtendStates.deployIntake;
    }

    public static void retractIntake(){
        States.intakeExtendState = IntakeExtendStates.retractIntake;
    }

    public static void feed() {
        States.intakeState = IntakeStates.feeding;
    }

    public static void stopIntake() {
        States.intakeState = IntakeStates.disabled;
    }

    public static void activateShooter() {
        States.shooterState = ShooterStates.preShoot;
    }

    public static void deactivateShooter() {
        States.shooterState = ShooterStates.disabled;
    }

    public static void intake() {
        if(States.intakeState != IntakeStates.feeding)
            States.intakeState = IntakeStates.intaking;
    }

    public static void outtake() {
        States.intakeState = IntakeStates.outtaking;
    }

    public static void stopClimber() {
        States.climberState = ClimberStates.disabled;
    }

    public static void extendClimberPiston() {
        States.climberState = ClimberStates.extendClimberPiston;
    }

    public static void retractClimberPiston() {
        States.climberState = ClimberStates.retractClimberPiston;
    }

    public static void extendClimber() {
        States.climberState = ClimberStates.extendClimber;
    }

    public static void retractClimber() {
        States.climberState = ClimberStates.retractClimber;
    }

    public static void fullClimb() {
        States.climberState = ClimberStates.fullClimb;
    }

}
