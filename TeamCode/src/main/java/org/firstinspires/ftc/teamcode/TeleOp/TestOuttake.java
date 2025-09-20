package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TestOuttake", group = "Mecanum")
public class TestOuttake extends LinearOpMode {
    private DcMotor outtake;
    private ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard ftcDashboard;


    @Override
    public void runOpMode() {

        this.ftcDashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initializing...");
        telemetry.update();


        outtake = hardwareMap.get(DcMotor.class,"outtake");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                outtake.setPower(1.0);
            }


            // Push telemetry to the dashboard
            telemetry.addData("Outtake Power: ", outtake.getPower());
            telemetry.update();



        }

    }

}