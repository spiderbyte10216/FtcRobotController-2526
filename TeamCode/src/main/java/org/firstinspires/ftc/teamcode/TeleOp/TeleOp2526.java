package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Limelight.TestBench;

@Config
@TeleOp(name = "TeleOp2526", group = "Mecanum")
public class TeleOp2526 extends LinearOpMode {
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private DcMotorEx intake;
    private CRServo indexer1;
    private CRServo indexer2;
    private CRServo indexer3;

    private PIDLauncher pidLauncher;

    private DigitalChannel laserInput;
    private ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard ftcDashboard;

    private boolean runOuttake1 = false;
    private boolean runOuttake2 = false;
    private boolean intakeMode = false;
    private boolean aWasPressed = false;

    private boolean beamBreakEnabled = true;

    public static double TARGET_VELOCITY_CLOSE = 450;
    public static double TARGET_VELOCITY_FAR = 525;
    public static double VELOCITY_TOLERANCE = 25;
    public static double NEW_P = 25;
    public static double NEW_I = 0.5;
    public static double NEW_D = 1.2;
    public static double NEW_F = 0.0;

    //public static int position = 0;
    private Limelight3A limelight3A;
    TestBench bench = new TestBench();

    private double distance;





    @Override
    public void runOpMode() {

        bench.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);

        limelight3A.start();
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
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");
        indexer3 = hardwareMap.get(CRServo.class, "indexer3");

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        laserInput.setMode(DigitalChannel.Mode.INPUT);

        pidLauncher = new PIDLauncher(runtime,telemetry);


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

        intake.setDirection(DcMotorSimple.Direction.FORWARD); //change direction if needed!

        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        PIDFCoefficients pidfOrig = outtake1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);




        indexer1.setPower(0);
        indexer2.setPower(0);
        indexer3.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();
        //rotateClaw.setPosition(rotateClawPosition);


        while (opModeIsActive()) {

            //get yaw from control hub IMU
            YawPitchRollAngles orientation = bench.getOrientation();
            limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            //get latest limelight result, pipeline 8 for April tag 20
            LLResult llResult = limelight3A.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botpose = llResult.getBotpose_MT2();
                distance = bench.getDistanceFromTage(llResult.getTa());
                telemetry.addData("Calculated Distance", distance);
                telemetry.addData("Target X", llResult.getTx());
                telemetry.addData("Target Area", llResult.getTa());
                telemetry.addData("Botpose", botpose.toString());

            } else {
                telemetry.addLine("No valid result");
            }
            telemetry.update();
            //move it back to initialization
            PIDFCoefficients outtake1PIDFCoefficientsNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            outtake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, outtake1PIDFCoefficientsNew);

            PIDFCoefficients outtake2PIDFCoefficientsNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            outtake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, outtake2PIDFCoefficientsNew);

            // Read the sensor state (true = HIGH, false = LOW)
            boolean stateHigh = laserInput.getState();

            // Active-HIGH: HIGH means an object is detected
            boolean detected = beamBreakEnabled && stateHigh;
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

            // Detect rising edge of button (press, not hold)
            if (gamepad2.a && !aWasPressed) {

                intakeMode = !intakeMode;   // toggle
                beamBreakEnabled = true;
            }

            aWasPressed = gamepad2.a;  // remember last state


            // INTAKE MODE (only when not shooting)
            if (intakeMode && !runOuttake1) {

                // intake always on in intakeMode
                intake.setPower(0.8);

                // indexer1 always on in intakeMode
                indexer1.setPower(1.0);

                // your rule:
                // nothing detected  -> indexer2 ON
                // something detected -> indexer2 OFF
                if (detected) {
                    indexer2.setPower(0.0);
                    indexer3.setPower(0.0);
                    telemetry.addLine("Object detected!");
                } else {
                    indexer2.setPower(1.0);
                    indexer3.setPower(-1.0);
                    telemetry.addLine("No object detected");
                }

            } else if (!runOuttake1) {
                // OFF mode when not shooting and not intaking
                intake.setPower(0.0);
                indexer1.setPower(0.0);
                indexer2.setPower(0.0);
                indexer3.setPower(0.0);
            }



            // --- INPUTS ---
            if (gamepad2.dpad_up) {
                runOuttake1 = true;   // start spinning flywheel
                runOuttake2 = false;
                beamBreakEnabled = false;

            }

            if (gamepad2.dpad_down) {
                runOuttake2 = true;   // start spinning flywheel
                runOuttake1=false;
                beamBreakEnabled = false;

            }

            if (gamepad2.y) {
                runOuttake1 = false;  // stop everything
            }

            if (gamepad2.x) {
                runOuttake2 = false;  // stop everything
            }

// --- CONTROL LOGIC ---
            boolean runOuttake = runOuttake1 || runOuttake2;

            if (runOuttake) {
                double targetVelocity = runOuttake2 ? TARGET_VELOCITY_FAR : TARGET_VELOCITY_CLOSE;

                outtake1.setVelocity(targetVelocity);
                outtake2.setVelocity(targetVelocity);

                double v1 = outtake1.getVelocity();
                double v2 = outtake2.getVelocity();
                double avgVelocity = (v1 + v2) / 2;

                boolean atSpeed = Math.abs(avgVelocity - targetVelocity) <= VELOCITY_TOLERANCE;

                if (atSpeed) {
                    indexer1.setPower(1);
                    indexer2.setPower(1);
                    indexer3.setPower(-1);
                    intake.setPower(1);
                } else {
                    indexer1.setPower(0);
                    indexer2.setPower(0);
                    indexer3.setPower(0);
                }
            } else {
                outtake1.setPower(0);
                outtake2.setPower(0);
            }


            if ((!runOuttake1 && !runOuttake2) && gamepad2.dpad_right) {
                indexer1.setPower(1.0);
                indexer2.setPower(1.0);
                indexer3.setPower(-1.0);
            }

            if ((!runOuttake1 && !runOuttake2) && gamepad2.dpad_left) {
                outtake1.setPower(0.55);
                outtake2.setPower(0.55);
            }



            // Push telemetry to the dashboard
            telemetry.addData("Outtake Power1: ", outtake1.getPower());
            telemetry.addData("Outtake Power2: ", outtake2.getPower());
            telemetry.addData("Intake Power: ", intake.getPower());
            telemetry.addData("Indexer1 Power: ", indexer1.getPower());
            telemetry.addData("Indexer2 Power: ", indexer2.getPower());
            telemetry.addData("Indexer3 Power: ", indexer3.getPower());
            // Display the raw HIGH/LOW signal for reference
            telemetry.addData("Raw (HIGH/LOW)", stateHigh);
            telemetry.addData("Target Velocity",TARGET_VELOCITY_CLOSE);
            telemetry.addData("Target Velocity",TARGET_VELOCITY_FAR);
            telemetry.addData("Velocity1",outtake1.getVelocity());
            telemetry.addData("Velocity2",outtake2.getVelocity());

            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.update();



        }

    }


}