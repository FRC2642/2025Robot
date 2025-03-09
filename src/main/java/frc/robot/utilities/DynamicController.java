// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;

/** The Interpolator Class uses basic math to smooth values. <br>
 * For more information contact Aditya Meher.<br>
 * There are multiple constructors avaliable for initializing a DynamicController. <br>
*/
public class DynamicController {
    private ArrayList<Double> prevOutputs = new ArrayList<Double>();
    private int listMax;
    private double modifier;

    private boolean cutValue = false;
    private double outputMin = -1;
    private double outputMax = 1;

    /**
     * Create a new Interpolator.
     * @param potential The amount of control the DynamicController should calculate. It is recommended to use {@link #DynamicController(int, double)} (or similar) instead to set the parameters that are calculated from this value.
     * @param limitValue Whether or not to limit the value. Use {@link #setMinMax(double, double)} to set the min & max and {@link #disableCutting(boolean)} to disable it entirely. Defaults to -1 and 1.
     * @throws IllegalArgumentException if potential <= 0 or >= 1
     */
    public DynamicController(double potential, boolean limitValue) { // Initialize an Interpolator
        if (potential <= 0 || potential >= 1) throw new IllegalArgumentException();
        setPotential(potential);
        this.cutValue = limitValue;
    }
    public DynamicController(int maxIterations, double percentModifier) {
        setModifiers(maxIterations, percentModifier);
    }
    public DynamicController(int maxIterations, double percentModifier, boolean limitValue) {
        setModifiers(maxIterations, percentModifier);
        this.cutValue = limitValue;
    }
    public DynamicController(int maxIterations, double percentModifier, double min, double max) {
        setModifiers(maxIterations, percentModifier);
        this.outputMin = min;
        this.outputMax = max;
    }

    /**
     * @param potential the amount of control the DynamicController should calculate
     * @throws IllegalArgumentException if potential <= 0 or >= 1
     */
    public void setPotential(double potential) {
        if (potential <= 0 || potential >= 1) throw new IllegalArgumentException();

        this.listMax = (int) potential * 300;
        this.modifier = 1 - potential;
    }

    /**
     * For customizing the Controller.
     * @param maxIteratons Maximum number of previous values for the controller to record.
     * @param percentModifier The percent to modify outputs by.
     * @throws IllegalArgumentException if maxIterations <= 1.
     * @throws IllegalArgumentException if percentModifier <= 0 or >= 1.
     */
    public void setModifiers(int maxIteratons, double percentModifier) {
        if (maxIteratons <= 1) throw new IllegalArgumentException();
        if (percentModifier <= 0 || percentModifier >= 1) throw new IllegalArgumentException();

        this.listMax = maxIteratons;
        this.modifier = percentModifier;
    }

    /**
     * Sets the Controller's min and maximum values.
     * @param min
     * @param max
     */
    public void setMinMax(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
        cutValue = true;
    }

    public void disableCutting(boolean cut) {
        cutValue = !cut;
    }

    /**
     * Update the Controller with a new output. ONLY USE IF THE CONTROLLER IS IDLE.
     * @param value Input 0 when idle. Otherwise, input the current output.
     */
    public void updateOutputs(double value) {
        prevOutputs.add(value);
        if (prevOutputs.size() > listMax) prevOutputs.remove(0);
    }

    /**
     * Calcuates and returns 
     * @param desiredValue
     */
    public double calculateOutput(double currentValue, double desiredValue) {
        double output = (currentValue - desiredValue) * modifier;
        if (cutValue) output = MathUtil.clamp(output, outputMin, outputMax);
        updateOutputs(output);
        
        double average = 0;
        for (int i = 0; i < prevOutputs.size(); i++) average += prevOutputs.get(i);
        average /= prevOutputs.size();

        if (Math.abs(output) > Math.abs(average)) return average;
        else return output;
    }
}
