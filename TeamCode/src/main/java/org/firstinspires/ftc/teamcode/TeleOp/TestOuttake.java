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
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "TestOuttake", group = "Mecanum")
public class TestOuttake extends LinearOpMode {

    private DcMotor intake;
    private ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard ftcDashboard;


    @Override
    public void runOpMode() {

        this.ftcDashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initializing...");
        telemetry.update();


        intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                intake.setPower(0.8);
            }
            if (gamepad1.y) {
                intake.setPower(0);
            }

           /* if (gamepad1.b){
                indexer1.setPower(-1.0);
                indexer2.setPower(-1.0);
            } */


            // Push telemetry to the dashboard
            telemetry.addData("Intake Power: ", intake.getPower());
            telemetry.update();



        }

    }

}