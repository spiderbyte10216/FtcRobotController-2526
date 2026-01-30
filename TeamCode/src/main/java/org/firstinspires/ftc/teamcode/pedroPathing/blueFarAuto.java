package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;  // use Autonomous for auto
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.TeleOp.PIDLauncher;

@Autonomous(name = "Blue Far Auto", group = "Pedro")
public class blueFarAuto extends OpMode {

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private DcMotorEx intake;
    private CRServo indexer1;
    private CRServo indexer2;

    private PIDLauncher pidLauncher;

    private DigitalChannel laserInput;

    private boolean runOuttake = false;
    private boolean intakeMode = false;
    private boolean aWasPressed = false;

    private boolean feedLatched = false;
    private static double TARGET_VELOCITY = 560;
    private static double TARGET_VELOCITY1 = 535;
    private static final double VELOCITY_TOLERANCE = 20;
    private static final double VELOCITY_TOLERANCE1 = 25;
    public static double NEW_P = 25;
    public static double NEW_I = 0.5;
    public static double NEW_D = 1.2;
    public static double NEW_F = 0.0;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        FIRST_LINE,

        THROUGH_FIRST_LINE,
        BACK_TO_SHOOT1,

        SHOOT_FIRST,
        READY_TELE
    }

    PathState pathState;

    private final Pose startPose = new Pose(56,8, Math.toRadians(90));
    private final Pose shootPose = new Pose(64.7077267637178,16.004479283314673,Math.toRadians(115));




    private final Pose firstLine = new Pose(38.7458006718925, 35.51399776035834, Math.toRadians(180));
    private final Pose throughFirstLine = new Pose(11.558790593505043, 35.51399776035834, Math.toRadians(180));

    private final Pose readyTele = new Pose(45.54983202687569, 30.2889137737962, Math.toRadians(115));
    private PathChain driveStartPosShootPos;

    private PathChain driveFirstLinePos;

    private PathChain driveThroughFirstLinePos;

    private PathChain driveReadyTelePos;

    private PathChain driveBackToShootPos1;


    private boolean startedFirstPath = false;
    private boolean startedSecondPath = false;

    private boolean startedThirdPath = false;
    private boolean startedFourthPath = false;

    private boolean startedFifthPath = false;


    public void buildPaths(){
        //put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveFirstLinePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,firstLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstLine.getHeading())
                .build();

        driveThroughFirstLinePos = follower.pathBuilder()
                .addPath(new BezierLine(firstLine,throughFirstLine))
                .setLinearHeadingInterpolation(firstLine.getHeading(), throughFirstLine.getHeading())
                .build();

        driveBackToShootPos1 = follower.pathBuilder()
                .addPath(new BezierLine(throughFirstLine, shootPose))
                .setLinearHeadingInterpolation(throughFirstLine.getHeading(), shootPose.getHeading())
                .build();

        driveReadyTelePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,readyTele))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyTele.getHeading())
                .build();


    }

    private void doShootPreload() {
        // Read sensor
        boolean stateHigh = laserInput.getState();
        boolean detected = stateHigh;  // HIGH = object present

        // In auto we always want to shoot in this state
        runOuttake = true;

        if (runOuttake) {
            // 1) Spin up outtake to target velocity
            outtake1.setVelocity(TARGET_VELOCITY);
            outtake2.setVelocity(TARGET_VELOCITY);

            // 2) Check actual velocity (average)
            double v2 = outtake2.getVelocity();
            double v1 = outtake1.getVelocity();
            double avgVelocity = (v1 + v2) / 2;
            boolean atSpeed = Math.abs(avgVelocity - TARGET_VELOCITY) <= VELOCITY_TOLERANCE;

            // 3) Intake + indexer logic

            // We'll mimic your TeleOp behavior:
            //  - intake ON
            //  - indexer1 always ON while shooting
            //  - indexer2 stops when laser sees a pixel (to avoid jamming), otherwise runs

            intake.setPower(1.0);
            indexer1.setPower(1.0);

            if (atSpeed) feedLatched = true;

            if (feedLatched) {
                indexer1.setPower(1.0);
                indexer2.setPower(1.0);
                intake.setPower(1.0);
            } else {
                // not up to speed yet, don't feed
                indexer1.setPower(0.0);
                indexer2.setPower(0.0);
                intake.setPower(0.0);
            }

            telemetry.addData("Indexer1 Power: ", indexer1.getPower());
            telemetry.addData("Indexer2 Power: ", indexer2.getPower());

            telemetry.addData("Target Velocity",TARGET_VELOCITY);
            telemetry.addData("Velocity1",outtake1.getVelocity());
            telemetry.addData("Velocity2",outtake2.getVelocity());

            telemetry.update();
            // 4) End condition: after some time, stop and move on
            // pathTimer was reset when we entered SHOOT_PRELOAD in setPathState()
            if (pathTimer.getElapsedTimeSeconds() > 11.0) {  // tweak for how long to shoot
                // stop shooter and feeds
                runOuttake = false;
                //outtake1.setVelocity(0);
                //outtake2.setVelocity(0);
                intake.setPower(0.0);
                indexer1.setPower(0.0);
                indexer2.setPower(0.0);

                // go to next path
                setPathState(PathState.FIRST_LINE);
            }

            // Debug telemetry
            telemetry.addData("SHOOT_PRELOAD atSpeed", atSpeed);
            telemetry.addData("Outtake v1", v1);
            telemetry.addData("Laser detected", detected);
        }
    }

    private void doShootPreload1() {
        // Read sensor
        boolean stateHigh = laserInput.getState();
        boolean detected = stateHigh;  // HIGH = object present

        // In auto we always want to shoot in this state
        runOuttake = true;

        if (runOuttake) {
            // 1) Spin up outtake to target velocity
            outtake1.setVelocity(TARGET_VELOCITY1);
            outtake2.setVelocity(TARGET_VELOCITY1);

            // 2) Check actual velocity (average)
            double v2 = outtake2.getVelocity();
            double v1 = outtake1.getVelocity();
            double avgVelocity = (v1 + v2) / 2;
            boolean atSpeed = Math.abs(avgVelocity - TARGET_VELOCITY1) <= VELOCITY_TOLERANCE1;

            // 3) Intake + indexer logic

            // We'll mimic your TeleOp behavior:
            //  - intake ON
            //  - indexer1 always ON while shooting
            //  - indexer2 stops when laser sees a pixel (to avoid jamming), otherwise runs

            intake.setPower(1.0);
            indexer1.setPower(1.0);

            if (atSpeed) feedLatched = true;

            if (feedLatched) {
                indexer1.setPower(1.0);
                indexer2.setPower(1.0);
                intake.setPower(1.0);
            } else {
                // not up to speed yet, don't feed
                indexer1.setPower(0.0);
                indexer2.setPower(0.0);
                intake.setPower(0.0);
            }

            telemetry.addData("Indexer1 Power: ", indexer1.getPower());
            telemetry.addData("Indexer2 Power: ", indexer2.getPower());

            telemetry.addData("Target Velocity",TARGET_VELOCITY1);
            telemetry.addData("Velocity1",outtake1.getVelocity());
            telemetry.addData("Velocity2",outtake2.getVelocity());

            telemetry.update();

            /*if (detected) {
                // object blocking laser -> pause indexer2 to avoid double-feeding
                indexer2.setPower(0.0);
            } else {
                // no object in laser beam -> keep feeding
                indexer2.setPower(1.0);
            }*/

            // Optionally: only feed once flywheel is near speed
            /*if (!atSpeed) {
                // If you want to be conservative, comment this out if it's over-restrictive
                indexer1.setPower(0.0);
                indexer2.setPower(0.0);
            }*/

            // 4) End condition: after some time, stop and move on
            // pathTimer was reset when we entered SHOOT_PRELOAD in setPathState()
            if (pathTimer.getElapsedTimeSeconds() > 10.0) {  // tweak for how long to shoot
                // stop shooter and feeds
                runOuttake = false;
                //outtake1.setVelocity(0.0);
                //outtake2.setVelocity(0.0);
                intake.setPower(0.0);
                indexer1.setPower(0.0);
                indexer2.setPower(0.0);

                // go to next path
                setPathState(PathState.READY_TELE);
            }

            // Debug telemetry
            telemetry.addData("SHOOT_PRELOAD atSpeed", atSpeed);
            telemetry.addData("Outtake v1", v1);
            telemetry.addData("Laser detected", detected);
        }
    }

    // Runs the same "intake mode" logic you use in TeleOp
    private void runAutoIntakeMode() {
        // Read the sensor state (true = HIGH, false = LOW)
        boolean stateHigh = laserInput.getState();

        // Active-HIGH: HIGH means an object is detected
        boolean detected = stateHigh;

        // In auto, intake mode is only used when we are NOT shooting
        // intake always on in intakeMode
        intake.setPower(1.0);

        // indexer1 always on in intakeMode
        indexer1.setPower(1.0);

        // your rule:
        // nothing detected  -> indexer2 ON
        // something detected -> indexer2 OFF
        if (detected) {
            indexer2.setPower(0.0);
            telemetry.addLine("Intake: Object detected!");
        } else {
            indexer2.setPower(1.0);
            telemetry.addLine("Intake: No object detected");
        }

    }

    // Simple helper to turn intake/indexers off
    private void stopIntakeAndIndexers() {
        intake.setPower(0.0);
        indexer1.setPower(0.0);
        indexer2.setPower(0.0);
    }

    private void cleanseIntake() {
        intake.setPower(-1.0);
        if (pathTimer.getElapsedTimeSeconds() > 1.0) {  // tweak for how long to shoot
            intake.setPower(0.0);
        }
    }


    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                spinUpOuttake();
                if (!startedFirstPath) {
                    follower.followPath(driveStartPosShootPos, true);
                    startedFirstPath = true;
                }

                if (!follower.isBusy()) {
                    telemetry.addLine("Finished Path 1");
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case SHOOT_PRELOAD:
                doShootPreload();
                break;

            case FIRST_LINE:
                cleanseIntake();
                spinUpOuttake1();
                // Just drive to the first line, no intake yet
                if (!startedSecondPath) {
                    follower.followPath(driveFirstLinePos, true);
                    startedSecondPath = true;
                }

                if (!follower.isBusy()) {
                    telemetry.addLine("Finished Path 2 (to FIRST_LINE)");
                    // As soon as we arrive, go into THROUGH_FIRST_LINE
                    // pathTimer will reset here
                    setPathState(blueFarAuto.PathState.THROUGH_FIRST_LINE);
                }
                break;

            case THROUGH_FIRST_LINE:
                // Turn on intake + indexers while going through the line
                runAutoIntakeMode();
                spinUpOuttake1();
                if (!startedThirdPath) {
                    follower.followPath(driveThroughFirstLinePos, true);
                    startedThirdPath = true;
                }

                // Stay in this state until:
                //  - path is finished AND
                //  - we've spent at least 5 seconds here
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    telemetry.addLine("Finished Path 3 + 5s intake through FIRST line");

                    // Turn everything off before going back to shoot
                    stopIntakeAndIndexers();

                    setPathState(blueFarAuto.PathState.BACK_TO_SHOOT1);
                }
                break;
            case BACK_TO_SHOOT1:
                // ✅ spin up while returning
                runAutoIntakeMode();
                spinUpOuttake1();

                if (!startedFourthPath) {
                    follower.followPath(driveBackToShootPos1, true);
                    startedFourthPath = true;
                }

                telemetry.addData("Outtake at speed?", outtakeAtSpeed1());

                if (!follower.isBusy()) {
                    telemetry.addLine("Back at shooting position!");
                    setPathState(blueFarAuto.PathState.SHOOT_FIRST);
                }
                break;
            case SHOOT_FIRST:
                runAutoIntakeMode();
                doShootPreload1();
                break;

            case READY_TELE:
                if (!startedFifthPath) {
                    follower.followPath(driveReadyTelePos, true);
                    startedFifthPath = true;
                }
                if (!follower.isBusy()) {
                    telemetry.addLine("Finished Path 5");
                }
                break;


            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }
    private void spinUpOuttake() {
        outtake1.setVelocity(TARGET_VELOCITY);
        outtake2.setVelocity(TARGET_VELOCITY);
    }

    private void spinUpOuttake1() {
        outtake1.setVelocity(TARGET_VELOCITY1);
        outtake2.setVelocity(TARGET_VELOCITY1);
    }

    private boolean outtakeAtSpeed() {
        double v1 = outtake1.getVelocity();
        double v2 = outtake2.getVelocity();
        double avg = (v1 + v2) / 2.0;
        return Math.abs(avg - TARGET_VELOCITY) <= VELOCITY_TOLERANCE;
    }

    private boolean outtakeAtSpeed1() {
        double v1 = outtake1.getVelocity();
        double v2 = outtake2.getVelocity();
        double avg = (v1 + v2) / 2.0;
        return Math.abs(avg - TARGET_VELOCITY1) <= VELOCITY_TOLERANCE;
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        feedLatched = false;
    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower((hardwareMap));
        //TODO add in any other init mechanisms
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        indexer1 = hardwareMap.get(CRServo.class, "indexer1");
        indexer2 = hardwareMap.get(CRServo.class, "indexer2");

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        laserInput.setMode(DigitalChannel.Mode.INPUT);

        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        indexer1.setPower(0);
        indexer2.setPower(0);

        PIDFCoefficients outtake1PIDFCoefficientsNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        outtake1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, outtake1PIDFCoefficientsNew);

        PIDFCoefficients outtake2PIDFCoefficientsNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        outtake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, outtake2PIDFCoefficientsNew);

        PIDFCoefficients pidfOrig = outtake1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }


    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }



}