package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDFController;

@Config
public class PIDLauncher {
    public double calculateVelocity(double position, double lastPosition, double time, double lastTime){
        return (position-lastPosition) / (time-lastTime);
    }

    private PIDFController pidfController;
    private double lastTime;
    private double lastPosition1;
    private double lastPosition2;
    private ElapsedTime runtime;
    private Telemetry telemetry;



    public static double targetVelocity = 0;
    public PIDLauncher(ElapsedTime runtime, Telemetry telemetry){
        pidfController = new PIDFController(0,0,0,0);
        pidfController.reset();
        lastPosition1 = 0;
        lastPosition2 = 0;
        lastTime = runtime.time();
        this.runtime = runtime;
        this.telemetry = telemetry;

    }
    public double update(double positionMotor1, double positionMotor2) {
        double time = this.runtime.time();
        double velocity1 = calculateVelocity(positionMotor1,lastPosition1,time, lastTime);
        double velocity2 = calculateVelocity(positionMotor2,lastPosition2,time, lastTime);


        double velocity = (velocity1 + velocity2) / 2;
        double power = pidfController.calculate(targetVelocity, velocity);
        if(power > 1) {
            power = 1;
        } else if (power < 0){
            power = 0;
        }

        lastPosition1 = positionMotor1;
        lastPosition2 = positionMotor2;
        lastTime = time;
        telemetry.addData("velocity", velocity);
        telemetry.addData("power", power);
        telemetry.addData("error", velocity - targetVelocity);
        telemetry.update();
        return power;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }


}
