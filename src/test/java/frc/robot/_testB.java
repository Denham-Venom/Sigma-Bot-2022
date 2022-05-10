package frc.robot;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.controller.PIDController;



public class _testB {

    PIDController thetaController;
    @Test
    public void testing(){
        thetaController = new PIDController(Constants.Swerve.thetaKP, Constants.Swerve.thetaKI, Constants.Swerve.thetaKD);
        print(thetaController.calculate(5.0, 0.0));
    }

    public static void print(Object printing) {
        System.out.println(printing);
    }
}