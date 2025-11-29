package org.firstinspires.ftc.teamcode.PID.control;

/**
 * Proportional, integral, and derivative (PID) gains used by [PIDFController].
 *
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 */
public class PIDCoefficients{
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;
        public PIDCoefficients(double kP,double kI,double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
}
