package org.firstinspires.ftc.teamcode.PID;

import static org.firstinspires.ftc.teamcode.PID.control.PIDFController.epsilonEquals;

import org.firstinspires.ftc.teamcode.PID.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.PID.control.PIDFController;
import com.qualcomm.robotcore.util.MovingStatistics;

public class VelocityPIDFController {
    private static final int ACCEL_SAMPLES = 3;
    private static final double VELOCITY_EPSILON = 20 + 1e-6;

    private PIDFController controller;

    private MovingStatistics accelSamples;

    private double lastPosition = Double.NaN;
    private double lastVelocity = Double.NaN;

    private double kV;
    private double kA;
    private double kStatic;

    private VelocityPIDFController() {}

    public VelocityPIDFController(PIDCoefficients pid) {
        this(pid, 0.0, 0.0, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV) {
        this(pid, kV, 0.0, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA) {
        this(pid, kV, kA, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA, double kStatic) {
        controller = new PIDFController(pid);
        accelSamples = new MovingStatistics(ACCEL_SAMPLES);

        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;

        reset();
    }

    public void setTargetAcceleration(double acceleration) {
        controller.setTargetVelocity(acceleration);
    }

    public void setTargetVelocity(double velocity) {
        controller.setTargetPosition(velocity);
    }

    private double calculateAccel(double measuredPosition, double measuredVelocity) {
        double dx = measuredPosition - lastPosition;
        if (dx != 0.0 && Math.abs(measuredVelocity - lastVelocity) > VELOCITY_EPSILON) {
            double accel = (measuredVelocity * measuredVelocity - lastVelocity * lastVelocity) / (2.0 * dx);

            lastPosition = measuredPosition;
            lastVelocity = measuredVelocity;

            accelSamples.add(accel);
        } else {
            accelSamples.add(0.0);
        }

        return accelSamples.getMean();
    }

    public double update(double measuredPosition, double measuredVelocity) {
        if (Double.isNaN(lastPosition)) {
            lastPosition = measuredPosition;
        }
        if (Double.isNaN(lastVelocity)) {
            lastVelocity = measuredVelocity;
        }

        double accel = calculateAccel(measuredPosition, measuredVelocity);

        double correction = controller.update(measuredVelocity, accel);
        double feedforward = calculateMotorFeedforward(
                controller.getTargetPosition(), controller.getTargetVelocity(), kV, kA, kStatic);
        return correction + feedforward;
    }

    private double calculateMotorFeedforward(double targetPosition, double targetVelocity, double kV, double kA, double kStatic) {
        double basePower = targetPosition * kV + targetVelocity * kA;
        if (epsilonEquals(basePower,0.0)) {
            return 0.0;
        } else {
            int sign;
            if(basePower >= 0) {
                sign = 1;
            } else {
                sign = -1;
            }
            return basePower + sign * kStatic;
        }
    }

    public void reset() {
        controller.reset();

        lastPosition = Double.NaN;
        lastVelocity = Double.NaN;
    }
}
