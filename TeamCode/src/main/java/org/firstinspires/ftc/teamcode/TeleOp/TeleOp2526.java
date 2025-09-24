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

@TeleOp(name = "TeleOp2526", group = "Mecanum")
public class TeleOp2526 extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    //private DcMotor outtake;
    private ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard ftcDashboard;


    @Override
    public void runOpMode() {

        this.ftcDashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize drivetrain motors
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

        //outtake = hardwareMap.get(DcMotor.class,"outtake");


        // Reset and configure encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set directions for mecanum drive
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //rotateClaw.setPosition(rotateClawPosition);

        while (opModeIsActive()) {
            // Read joystick values
            double y = gamepad1.left_stick_y;  // Forward/backward (inverted for forward)
            double x = -gamepad1.left_stick_x;   // Side-to-side (strafe)
            double r = -gamepad1.right_stick_x;  // Rotation

            // Calculate motor powers
            double p1 = y + x + r;  // Left Front
            double p2 = y - x - r;  // Right Front
            double p3 = y - x + r;  // Left Back
            double p4 = y + x - r;  // Right Back

            // Normalize motor powers if any exceed 1.0
            double maxval = max(abs(p1), max(abs(p2), max(abs(p3), abs(p4))));
            if (maxval > 1.0) {
                p1 /= maxval;
                p2 /= maxval;
                p3 /= maxval;
                p4 /= maxval;
            }

            // Set motor powers
            leftFront.setPower(p1);
            rightFront.setPower(p2);
            leftBack.setPower(p3);
            rightBack.setPower(p4);

//            if (gamepad1.a) {
//                outtake.setPower(0.5);
//            }


            // Push telemetry to the dashboard
            telemetry.update();



        }

    }

}