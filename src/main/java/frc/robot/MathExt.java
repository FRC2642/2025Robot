// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Additional math functions free for use. Made by Aditya Meher */

package frc.robot;

/** Add your docs here. */
public class MathExt {
    
    /**
     * Returns -1 if value is negative and 1 if value is positive.
     */
    public static double getSign(double value) {
        return value / Math.abs(value);
    }
    /**
     * Returns -1 if value is negative and 1 if value is positive.
     */
    public static int getSign(int value) {
        return (int) value / Math.abs(value);
    }

    /**
     * Limits a value between the min and the max and returns the limited value.
     */

    public static double cutValue(double value, double min, double max) {
        if (value > max) { return max; }
        else if (value < min) { return min; }
        return value;
    }

    /**
     * Modifies the axial input, taking in an additional input to modify the original. Returns the result.
     * 
     * @param input The original input
     * @param modifierInput The secondary, modifying input
     * @param modifyPercent The percent of the value of the original input to be affected by the modifierInput
     */

    public static double modifyAxialInput(double input, double modifierInput, double modifyPercent) {
        input = cutValue(input, -1, 1);
        modifierInput = cutValue(modifierInput, 0, 1);
        double output = input * (modifierInput * modifyPercent + (1 - modifyPercent));
        // If the input is negative, made the modifier negative, and same for positive
        return output;
    }
}