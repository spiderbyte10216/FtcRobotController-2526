package org.firstinspires.ftc.teamcode.TeleOp;

//math functions
import static java.lang.Math.abs;
import static java.lang.Math.max;

//FTC Dashboard libraries
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;

//core FTC robot control classes
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//Limelight libraries
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

//navigation and orientation classes
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Limelight.TestBench;

//enable FTC Dashboard
@Config
@TeleOp(name = "RedTeleOp2526", group = "Mecanum")
public class RedTeleOp2526 extends LinearOpMode {

    //drivetrain motors
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    //flywheel motors
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;

    private DcMotorEx intake; //intake motor

    //indexer continuous rotation servos
    private CRServo indexer1;
    private CRServo indexer2;
    private CRServo indexer3;

    private PIDLauncher pidLauncher; //PID controller for flywheels

    private DigitalChannel laserInput; //Beam break sensor

    private ElapsedTime runtime = new ElapsedTime(); //Timer for tracking elapsed time during match

    public FtcDashboard ftcDashboard; //FTC Dashboard instance

    private boolean intakeMode = false; //intake mode toggle

    private boolean aWasPressed = false; //button state tracking to detect gamepad2.a button press

    private boolean beamBreakEnabled = true; //enable/disable beam break

    public static double VELOCITY_TOLERANCE = 25; //tolerance range for considering flywheel atSpeed

    //PIDF coefficients for flywheel velocity
    public static double NEW_P = 25;
    public static double NEW_I = 0.5;
    public static double NEW_D = 1.2;
    public static double NEW_F = 0.0;

    private Limelight3A limelight3A; //declare limelight camera

    TestBench bench = new TestBench(); //TestBench helper class for Limelight calculations

    private double distance; //distance to target (goal apriltags)

    private double targetFlywheel = 0; // current target RPM for the flywheel

    private double minTableRpm = 0;  //minimum RPM from the lookup table

    private boolean shootRequested = false; // indicates shooter should be actively feeding

    // toggle for enabling/disabling shooting mode
    private boolean shootEnabled = false;
    private boolean upWasPressed = false;

    // flag to stop/start the flywheels entirely
    private boolean outtakesStopped = false;
    private boolean rightTriggerWasPressed = false;

    private boolean hasValidTag = false; // flag indicating if the apriltag is currently visible

    // Emergency stop for the entire feed system
    private boolean feedStopped = false;
    private boolean yWasPressed = false;


    // -----NEW AUTO ALIGN!!

    // Toggle for automatic alignment to apriltag
    private boolean autoAlignEnabled = false;
    private boolean alignWasPressed = false;

    // auto-align tuning parameters
    public static double AIM_KP = 0.5;        // tune
    public static double AIM_TOLERANCE = 1.0;   // degrees, tune
    public static double AIM_MAX_POWER = 0.60;  // cap turn power

    public static double TX_OFFSET_DEG = -9.0;

    private boolean autoAlignLocked = false;   // prevents re-arming until B is released
    private int alignStableCount = 0;          // counts consecutive loops within tolerance

    public static int AIM_STABLE_LOOPS = 5;    // tune: how many loops to confirm "done"


    // DISTANCE-TO-RPM LOOKUP TABLE
    /**
     * * Table mapping distance (in cm) to required flywheel RPM
     * Format: { distance_in_cm, rpm_needed }
     * Used for automatic shot adjustment based on distance to target (calculated from limelight)
     */
    private static final double[][] DIST_RPM_TABLE = new double[][]{
            // { distance, rpm }
            { 91.0, 410 },
            { 112.0, 420 },
            { 129.0, 420 },
            { 144.0, 425 },
            { 175.0, 425 },
            { 207.0, 430 },
            { 230.0, 435 },
            { 262.0, 435 },
            { 295.0, 440 },
            { 373.0, 515 },
            { 400.0, 520 },
            { 446.0, 520 }
    };


    /**
     * Interpolates RPM value based on distance using the lookup table
     * @param dist Distance to target in cm
     * @return Interpolated RPM value
     */
    private static double rpmInterpolate(double dist) {
        // if distance is below minimum in table, use minimum RPM
        if (dist <= DIST_RPM_TABLE[0][0]) {
            return DIST_RPM_TABLE[0][1];
        }
        // if distance is above maximum in table, use maximum RPM
        if (dist >= DIST_RPM_TABLE[DIST_RPM_TABLE.length - 1][0]) {
            return DIST_RPM_TABLE[DIST_RPM_TABLE.length - 1][1];
        }

        // linear interpolation between two table entries
        for (int i = 0; i < DIST_RPM_TABLE.length - 1; i++) {
            double d0 = DIST_RPM_TABLE[i][0]; // lower distance bound
            double d1 = DIST_RPM_TABLE[i + 1][0]; // upper distance bound

            // check if current distance falls between these two entries
            if (dist >= d0 && dist <= d1) {
                double r0 = DIST_RPM_TABLE[i][1]; // RPM at lower bound
                double r1 = DIST_RPM_TABLE[i + 1][1]; //RPM at upper bound

                // calculate interpolation factor (0 to 1)
                double t = (dist - d0) / (d1 - d0);

                // liner interpolation formula
                return r0 + t * (r1 - r0);
            }
        }

        // fallback: return maximum RPM
        return DIST_RPM_TABLE[DIST_RPM_TABLE.length-1][1];
    }


    @Override
    public void runOpMode() {

        // initialize the TestBench helper (for IMU and limelight calculations)
        bench.init(hardwareMap);

        // add limelight to hardware map
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        //switch to limelight pipeline 8
        limelight3A.pipelineSwitch(7);

        //start the limelight
        limelight3A.start();

        //initialize FTC dashboard for telemetry
        this.ftcDashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // display initialization status
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // add drivetrain motors to hardware map
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // add flywheel motors to hardware map
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        // add intake motor to hardware map
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // add indexer CR servos to hardware map
        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");
        indexer3 = hardwareMap.get(CRServo.class, "indexer3");

        // add beam break sensor to hardware map
        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        // configure beam break sensor as input
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        // initialize PID controller for launcher
        pidLauncher = new PIDLauncher(runtime,telemetry);

        // set all drivetrain motors to run without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set motor directions for mecanum drive
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // set intake motor direction
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // set flywheel motors to run using encoders
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get original PIDF coefficients (for telemetry comparison)
        PIDFCoefficients pidfOrig = outtake1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // apply custom PIDF tuning to first flywheel motor
        PIDFCoefficients outtake1PIDFCoefficientsNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        outtake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, outtake1PIDFCoefficientsNew);

        // apply custom PIDF tuning to second flywheel motor
        PIDFCoefficients outtake2PIDFCoefficientsNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        outtake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, outtake2PIDFCoefficientsNew);

        // stop all indexer servos initially
        indexer1.setPower(0);
        indexer2.setPower(0);
        indexer3.setPower(0);

        // set minimum RPM from the lookup table
        minTableRpm = DIST_RPM_TABLE[0][1];

        // initialize flywheel target to minimum RPM
        targetFlywheel = minTableRpm;

        // display final initialization status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //get yaw from control hub IMU
            YawPitchRollAngles orientation = bench.getOrientation();
            limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            // get latest limelight result, pipeline 8 for April tag 20
            LLResult llResult = limelight3A.getLatestResult();

            // check if a valid target was detected
            boolean tagValid = (llResult != null && llResult.isValid());

            // horizontal offset to target (in degrees)
            double tx = 0.0;

            if (tagValid) {
                // get 3D pose of robot relative to target
                Pose3D botpose = llResult.getBotpose_MT2();

                // calculate distance to target based on target area
                distance = bench.getDistanceFromTage(llResult.getTa());

                // get horizontal offset for auto-alignment
                tx = llResult.getTx();

                // interpolate required RPM based on distance
                targetFlywheel = rpmInterpolate(distance);

                // mark that there is a valid target (april tag detected)
                hasValidTag = true;

                // limelight telemetry
                telemetry.addData("Calculated Distance", distance);
                telemetry.addData("RPM from table",targetFlywheel);
                telemetry.addData("Target X", llResult.getTx());
                telemetry.addData("Target Area", llResult.getTa());
                telemetry.addData("Botpose", botpose.toString());

            } else {
                // no valid target detected
                hasValidTag = false;

                // fallback to min RPM
                targetFlywheel = minTableRpm;

                telemetry.addLine("No valid result");
            }


            if (outtakesStopped) {
                // Emergency stop for flywheels
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
            } else {
                // Normal operation
                outtake1.setVelocity(targetFlywheel);
                outtake2.setVelocity(targetFlywheel);
            }
            telemetry.update();


            // Read the sensor state (true = HIGH, false = LOW)
            boolean stateHigh = laserInput.getState();

            // Active-HIGH: HIGH means an object is detected
            boolean detected = beamBreakEnabled && stateHigh;

            // Read joystick values
            double y = gamepad1.left_stick_y;  // Forward/backward (inverted for forward)
            double x = -gamepad1.left_stick_x;   // Side-to-side (strafe)

            // check if gamepad1.b button is pressed (start auto-align)
            boolean alignPressed = gamepad1.b;

            // unlock once driver releases B
            if (!alignPressed) {
                autoAlignLocked = false;
            }

            // only allow a new align if:
            // 1) B was just pressed (rising edge)
            // 2) we are not already aligning
            // 3) we are not locked (meaning we haven't released since last completion)
            if (alignPressed && !alignWasPressed) {
                if (!autoAlignEnabled && !autoAlignLocked) {
                    autoAlignEnabled = true;
                    alignStableCount = 0; // reset stability counter
                }
            }

            alignWasPressed = alignPressed;

            double rDriver = -gamepad1.right_stick_x;  // driver rotation
            double r = rDriver; // actual rotation to apply

            // AUTO-ALIGN LOGIC
            if (autoAlignEnabled) {
                if (tagValid) {

                    double txCorrected = tx - TX_OFFSET_DEG;

                    double turn = -AIM_KP * txCorrected;

                    // clamp
                    if (turn > AIM_MAX_POWER) turn = AIM_MAX_POWER;
                    if (turn < -AIM_MAX_POWER) turn = -AIM_MAX_POWER;

                    r = turn;

                    // require being within tolerance for a few consecutive loops
                    if (Math.abs(txCorrected) <= AIM_TOLERANCE) {
                        alignStableCount++;
                    } else {
                        alignStableCount = 0;
                    }

                    // done aligning
                    if (alignStableCount >= AIM_STABLE_LOOPS) {
                        autoAlignEnabled = false;
                        autoAlignLocked = true;   // <-- this is the key: prevents immediate re-align
                        r = 0.0;
                    }

                    telemetry.addData("txCorrected", txCorrected);
                    telemetry.addData("StableCount", alignStableCount);

                } else {
                    // lost tag -> exit align (and don't lock)
                    autoAlignEnabled = false;
                    alignStableCount = 0;
                    r = rDriver;
                }
            }


            telemetry.addData("AutoAlign", autoAlignEnabled);
            telemetry.addData("tx", tx);



            // Calculate motor powers
            double p1 = y + x + r;  // Left Front
            double p2 = y - x - r;  // Right Front
            double p3 = y - x + r;  // Left Back
            double p4 = y + x - r;  // Right Back

            // Normalize motor powers if any exceed [-1.0, 1.0]
            double maxval = max(abs(p1), max(abs(p2), max(abs(p3), abs(p4))));
            if (maxval > 1.0) {
                p1 /= maxval;
                p2 /= maxval;
                p3 /= maxval;
                p4 /= maxval;
            }

            // apply calculated powers
            leftFront.setPower(p1);
            rightFront.setPower(p2);
            leftBack.setPower(p3);
            rightBack.setPower(p4);

            // detect rising edge of gamepad2.a button (press, not hold)
            if (gamepad2.a && !aWasPressed) {
                intakeMode = !intakeMode; // toggle intake mode
                beamBreakEnabled = true; // re-enable beam break when entering intake mode
                feedStopped = false; // clear feed stop flag
            }

            aWasPressed = gamepad2.a;  // remember last state

            // detect rising edge of gamepad2.dpad_up button (press, not hold)
            boolean upPressed = gamepad2.dpad_up;
            if (upPressed && !upWasPressed) {
                shootEnabled = !shootEnabled; // toggle shooting enabled
            }
            upWasPressed = upPressed;

            shootRequested = shootEnabled; // set shoot request based on toggle state

            // disable beam break when shooting (so indexers run)
            if (shootEnabled) {
                beamBreakEnabled = false;
            }

            // INTAKE MODE (only when not shooting)
            if (intakeMode && !shootRequested) {
                // intake always on in intakeMode
                intake.setPower(1.0);

                // indexer1 always on in intakeMode
                indexer1.setPower(1.0);

                // Conditional control of indexers 2 & 3 based on beam break
                if (detected) {
                    // Object detected: STOP feeding
                    indexer2.setPower(0.0);
                    indexer3.setPower(0.0);
                    telemetry.addLine("Object detected!");
                } else {
                    // No object: CONTINUE feeding
                    indexer2.setPower(1.0);
                    indexer3.setPower(-1.0);
                    telemetry.addLine("No object detected");
                }

            } else if (!shootRequested) {
                // OFF mode when not shooting and not intaking
                intake.setPower(0.0);
                indexer1.setPower(0.0);
                indexer2.setPower(0.0);
                indexer3.setPower(0.0);
            }

            // --- VELOCITY LOGIC ---

            // get current velocities of both flywheels
            double v1 = outtake1.getVelocity();
            double v2 = outtake2.getVelocity();

            // calculate average velocity
            double avgVelocity = (v1 + v2) / 2.0;

            // check if flywheels are at target speed (within tolerance)
            boolean atSpeed = Math.abs(avgVelocity - targetFlywheel) <= VELOCITY_TOLERANCE;

            // launcher telemetry
            telemetry.addData("Shooter Target", targetFlywheel);
            telemetry.addData("Shooter AvgVel", avgVelocity);
            telemetry.addData("Shooter AtSpeed", atSpeed);
            telemetry.addData("Tag Visible", hasValidTag);

            // --- SHOOTING LOGIC ---

            if (shootRequested && !outtakesStopped && atSpeed) {
                // shoot if shoot button was pressed and flywheels are at speed
                indexer1.setPower(1);
                indexer2.setPower(1);
                indexer3.setPower(-1);
                intake.setPower(1);
            } else if (!intakeMode){
                // stop all indexers
                indexer1.setPower(0);
                indexer2.setPower(0);
                indexer3.setPower(0);
            }

            // --- FLYWHEEL STOP TOGGLE ---
            boolean right_trigger = gamepad2.right_bumper;
            if (right_trigger && !rightTriggerWasPressed) {
                outtakesStopped = !outtakesStopped;
            }
            rightTriggerWasPressed = right_trigger;

            // --- EMERGENCY FEED STOP ---
            boolean yPressed = gamepad2.y;
            if (yPressed && !yWasPressed) {
                feedStopped = !feedStopped;

                if (feedStopped) {
                    intakeMode = false;
                    shootEnabled = false;
                }
            }
            yWasPressed = yPressed;

            // If emergency stop is active, force all feed motors off
            if (feedStopped) {
                intake.setPower(0);
                indexer1.setPower(0);
                indexer2.setPower(0);
                indexer3.setPower(0);

                telemetry.addData("FeedStopped", true);
                telemetry.update();
                continue; // Skip rest of loop (prevents other logic from overriding)
            }
            telemetry.addData("FeedStopped", false);



            // --- TELEMETRY OUTPUT ---

            // Display all relevant status information
            telemetry.addData("ShootEnabled", shootEnabled);
            telemetry.addData("Outtake Power1: ", outtake1.getPower());
            telemetry.addData("Outtake Power2: ", outtake2.getPower());
            telemetry.addData("Intake Power: ", intake.getPower());
            telemetry.addData("Indexer1 Power: ", indexer1.getPower());
            telemetry.addData("Indexer2 Power: ", indexer2.getPower());
            telemetry.addData("Indexer3 Power: ", indexer3.getPower());
            telemetry.addData("Raw (HIGH/LOW)", stateHigh);
            telemetry.addData("Velocity1",outtake1.getVelocity());
            telemetry.addData("Velocity2",outtake2.getVelocity());

            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);

            telemetry.update();

        }

    }


}