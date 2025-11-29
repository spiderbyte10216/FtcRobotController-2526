package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "TeleOp2526", group = "Mecanum")
public class TeleOp2526 extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private CRServo intake;
    private CRServo indexer1;
    private CRServo indexer2;

    private DigitalChannel laserInput;
    private ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard ftcDashboard;

    private static final double TICKS_PER_REV = 295.0;


    @Override
    public void runOpMode() {

        this.ftcDashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize drivetrain motors
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        intake = hardwareMap.get(CRServo.class, "intake");

        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        laserInput.setMode(DigitalChannel.Mode.INPUT);


        // Reset and configure encoders
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set directions for mecanum drive
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //rotateClaw.setPosition(rotateClawPosition);

        while (opModeIsActive()) {
            indexer1.setPower(0.0);
            indexer2.setPower(0.0);
            // Read the sensor state (true = HIGH, false = LOW)
            boolean stateHigh = laserInput.getState();

            // Active-HIGH: HIGH means an object is detected
            boolean detected = stateHigh;
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

            leftFront.setPower(p1);
            rightFront.setPower(p2);
            leftBack.setPower(p3);
            rightBack.setPower(p4);



            if (gamepad2.a) {
                intake.setPower(1.0);//indexer spin slow
                //indexer2.setPower(0.2);
            }
            if (gamepad2.b) {
                intake.setPower(0.0);
                indexer1.setPower(0.0); //indexer spin slow
                //indexer2.setPower(0.0);
            }
            if (gamepad2.dpad_down) {
                outtake1.setPower(0.3);
                outtake2.setPower(0.3);//THIS IS TO CHANGE THE FLYWHEEL POWER
                indexer1.setPower(1.0);
                indexer2.setPower(1.0);
            }

            if (gamepad2.dpad_up) {
                outtake1.setPower(0.3);
                outtake2.setPower(0.3);//THIS IS TO CHANGE THE FLYWHEEL POWER
                //indexer1.setPower(1.0);
                //indexer2.setPower(1.0);
            }

            if (gamepad2.x) {
                outtake1.setPower(0.0);
                outtake2.setPower(0.0);//THIS IS TO CHANGE THE FLYWHEEL POWER!
                indexer1.setPower(0.0); //indexer spin fast
                //indexer2.setPower(0.0);
            }

            if (gamepad2.right_bumper) {
                indexer1.setPower(1.0);
                if (detected) {
                    indexer2.setPower(0.0);
                    telemetry.addLine("Object detected!");
                } else {
                    indexer2.setPower(1.0);
                    telemetry.addLine("No object detected");
                }
            }

            if (gamepad2.left_bumper) {
                indexer1.setPower(-0.5);
                if (detected) {
                    indexer2.setPower(0.0);
                    telemetry.addLine("Object detected!");
                } else {
                    indexer2.setPower(-0.5);
                    telemetry.addLine("No object detected");
                }
            }

            if(gamepad2.dpad_right) {
                indexer1.setPower(1.0);
                indexer2.setPower(1.0);
            }


            double outtake1TicksPerSec = outtake1.getVelocity();
            double outtake2TicksPerSec = outtake2.getVelocity();

            double outtake1Rpm = (outtake1TicksPerSec * 60.0) / TICKS_PER_REV;
            double outtake2Rpm = (outtake2TicksPerSec * 60.0) / TICKS_PER_REV;

            // Push telemetry to the dashboard
            telemetry.addData("Outtake Power1: ", outtake1.getPower());
            telemetry.addData("Outtake Power2: ", outtake2.getPower());
            telemetry.addData("Outtake1 RPM", "%.1f", outtake1Rpm);
            telemetry.addData("Outtake2 RPM", "%.1f", outtake2Rpm);
            telemetry.addData("Intake Power: ", intake.getPower());
            telemetry.addData("Indexer1 Power: ", indexer1.getPower());
            telemetry.addData("Indexer2 Power: ", indexer2.getPower());
            // Display the raw HIGH/LOW signal for reference
            telemetry.addData("Raw (HIGH/LOW)", stateHigh);
            telemetry.update();



        }

    }

}