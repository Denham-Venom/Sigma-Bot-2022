// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.filter.MedianFilter;

/** Add your docs here. */
public class BooleanMedianFilter {
    private MedianFilter filter;
    
    public BooleanMedianFilter(int size) {
        filter = new MedianFilter(size);
    }

    public boolean calculate(boolean next) {
        double result;
        if(next) {
            result = filter.calculate(1);
        } else {
            result = filter.calculate(0);
        }

        int rounded = (int) Math.round(result);
        if(rounded > 0) return true;
        else return false;
    }
}
