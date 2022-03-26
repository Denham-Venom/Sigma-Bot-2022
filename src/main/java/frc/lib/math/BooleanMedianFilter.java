// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.filter.MedianFilter;

/** Add your docs here. */
public class BooleanMedianFilter extends MedianFilter {
    
    public BooleanMedianFilter(int size) {
        super(size);
    }

    public boolean calculate(boolean next) {
        double result;
        if(next) {
            result = super.calculate(1);
        } else {
            result = super.calculate(0);
        }

        int rounded = (int) Math.round(result);
        if(rounded > 0) return true;
        else return false;
    }

    @Override
    public double calculate(double next) {
        double result;
        if(next != 0) {
            result = calculate(true) ? 1 : 0;
        } else {
            result = calculate(false) ? 1 : 0;
        }
        return result;
    }
}
