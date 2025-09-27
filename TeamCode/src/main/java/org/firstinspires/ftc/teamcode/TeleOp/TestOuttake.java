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
    private DcMotor outtake;
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo indexer1;
    private CRServo indexer2;
    public FtcDashboard ftcDashboard;


    @Override
    public void runOpMode() {

        this.ftcDashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initializing...");
        telemetry.update();


        outtake = hardwareMap.get(DcMotor.class,"outtake");
        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");

        indexer1.setDirection(CRServo.Direction.FORWARD);
        indexer2.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                outtake.setPower(1);
            }

            if (gamepad1.b){
                indexer1.setPower(-1.0);
                indexer2.setPower(-1.0);
            }


            // Push telemetry to the dashboard
            telemetry.addData("Outtake Power: ", outtake.getPower());
            telemetry.update();



        }

    }

}