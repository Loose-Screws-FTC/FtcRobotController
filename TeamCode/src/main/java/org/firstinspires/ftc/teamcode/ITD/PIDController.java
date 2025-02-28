package org.firstinspires.ftc.teamcode.ITD;

import static java.lang.Double.min;

import java.util.ArrayList;
import java.util.Deque;
import java.util.ArrayDeque;

public class PIDController {
    public double P;
    public double I;
    public double D;
    private final double differentiateTime = 0.1;
    private double integral = 0;
    private double previousError;
    private final ArrayList<ErrorSample> samples;

    private class ErrorSample {
        public final double time;
        public final double error;

        public ErrorSample(double time, double error) {
            this.time = time;
            this.error = error;
        }
    }

    private double min;
    private double max;

    public PIDController(double P, double I, double D, double min, double max) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.min = min;
        this.max = max;
        this.samples = new ArrayList<>();
    }
    public double calculate(double target, double currentValue, double deltaTime) {
        double error = target - currentValue;

        integral += error * deltaTime;

         double now = System.currentTimeMillis() / 1000.0;

         samples.add(new ErrorSample(now, error));
         if (now - samples.get(0).time > differentiateTime) samples.remove(0);

        double derivativeSum = 0;
        for (int i = 0; i < samples.size() - 1; i++) {
            derivativeSum += samples.get(i+1).error - samples.get(i).error / (samples.get(i+1).time - samples.get(i).time);
        }

        double derivative = derivativeSum / (samples.size() - 1);
        
        // double derivative = (error - previousError) / deltaTime;
        previousError = error;
        double output = (P * error) + (I * integral) + (D * derivative);


        return normalizeToRange(output);
    }

    private double normalizeToRange(double value) {
        if (value > max) value = max;
        if (value < min) value = min;
        return value;
    }
}
