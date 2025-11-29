package org.firstinspires.ftc.teamcode.PID.control;

import java.util.function.BiFunction;

/**
 * PID controller with feedforward components (kV, kA, kStatic, custom kF).
 * This version DOES NOT use Road Runner's NanoClock — uses System.nanoTime().
 */
public class PIDFController {

    private final PIDCoefficients pid;
    private final double kV;
    private final double kA;
    private final double kStatic;
    private final BiFunction<Double, Double, Double> kF;

    private double errorSum = 0.0;
    private double lastUpdateTimestamp = Double.NaN;

    private boolean inputBounded = false;
    private double minInput = 0.0;
    private double maxInput = 0.0;

    private boolean outputBounded = false;
    private double minOutput = 0.0;
    private double maxOutput = 0.0;

    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;
    private double targetAcceleration = 0.0;
    private double lastError = 0.0;

    // --- helper to convert nanoTime to seconds ---
    private double now() {
        return System.nanoTime() / 1e9;
    }

    // ----- Constructors -----

    public PIDFController(PIDCoefficients pid) {
        this(pid, 0, 0, 0, (p, v) -> 0.0);
    }

    public PIDFController(PIDCoefficients pid, double kV, double kA, double kStatic) {
        this(pid, kV, kA, kStatic, (p, v) -> 0.0);
    }

    public PIDFController(
            PIDCoefficients pid,
            double kV,
            double kA,
            double kStatic,
            BiFunction<Double, Double, Double> kF
    ) {
        this.pid = pid;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = (kF != null ? kF : (p, v) -> 0.0);
    }

    // ----- Input/Output bounds -----

    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    public void setOutputBounds(double min, double max) {
        if (min < max) {
            outputBounded = true;
            minOutput = min;
            maxOutput = max;
        }
    }

    // ----- Position error handling -----

    private double getPositionError(double measuredPosition) {
        double error = targetPosition - measuredPosition;

        if (inputBounded) {
            double range = maxInput - minInput;
            while (Math.abs(error) > range / 2.0) {
                error -= Math.signum(error) * range;
            }
        }

        return error;
    }

    // ----- Update -----

    public double update(double measuredPosition) {
        return update(measuredPosition, null);
    }

    public double update(double measuredPosition, Double measuredVelocity) {
        double currentTime = now();
        double error = getPositionError(measuredPosition);

        if (Double.isNaN(lastUpdateTimestamp)) {
            lastUpdateTimestamp = currentTime;
            lastError = error;
            return 0.0;
        }

        double dt = currentTime - lastUpdateTimestamp;
        lastUpdateTimestamp = currentTime;

        errorSum += 0.5 * (error + lastError) * dt;

        double errorDeriv = (error - lastError) / dt;
        lastError = error;

        double derivTerm = (measuredVelocity != null)
                ? (targetVelocity - measuredVelocity)
                : errorDeriv;

        double baseOutput =
                pid.kP * error +
                        pid.kI * errorSum +
                        pid.kD * derivTerm +
                        kV * targetVelocity +
                        kA * targetAcceleration +
                        kF.apply(measuredPosition, measuredVelocity);

        double output;

        if (epsilonEquals(baseOutput, 0.0)) {
            output = 0.0;
        } else {
            output = baseOutput + Math.signum(baseOutput) * kStatic;
        }

        if (outputBounded) {
            output = Math.max(minOutput, Math.min(output, maxOutput));
        }

        return output;
    }

    // ----- Reset -----

    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastUpdateTimestamp = Double.NaN;
    }

    // ----- Getters/Setters -----

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double getTargetAcceleration() {
        return targetAcceleration;
    }

    public void setTargetAcceleration(double targetAcceleration) {
        this.targetAcceleration = targetAcceleration;
    }

    public double getLastError() {
        return lastError;
    }

    // ----- epsilonEquals replacement -----

    public static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < 1e-6;
    }
}
