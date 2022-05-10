package frc.robot;

import org.junit.jupiter.api.Test;

import frc.lib.math.Conversions;



public class _testA {

    @Test
    public void testing(){
        print(1023.0 / Conversions.RPMToFalcon(5700, Constants.Shooter.shooterGearRatio));
    }

    public static void print(Object printing) {
        System.out.println(printing);
    }
}