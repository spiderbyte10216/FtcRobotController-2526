package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name = "Blue Auto New", group = "Pedro")
public class blueAutoNew extends OpMode {

    /* ---------------- Hardware ---------------- */
    private DcMotor rightFront, leftFront, leftBack, rightBack;
    private DcMotorEx outtake1, outtake2, intake;
    private CRServo indexer1, indexer2;
    private DigitalChannel laserInput;

    /* ---------------- Shooter Control ---------------- */
    private static final double TARGET_VELOCITY = 400;
    private static final double VELOCITY_TOLERANCE = 50;

    private boolean lastDetected = false;
    private int shotsThisState = 0;

    /* ---------------- Pathing ---------------- */
    private Follower follower;
    private Timer pathTimer;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        FIRST_LINE,
        THROUGH_FIRST_LINE,
        BACK_TO_SHOOT1,
        SHOOT_FIRST,
        SECOND_LINE,
        THROUGH_SECOND_LINE,
        BACK_TO_SHOOT2,
        SHOOT_SECOND,
        READY_TELE
    }

    private PathState pathState;

    /* ---------------- Poses ---------------- */
    private final Pose startPose = new Pose(22,121.5, Math.toRadians(138));
    private final Pose shootPose = new Pose(54.6,88.5,Math.toRadians(138));
    private final Pose firstLine = new Pose(42.4,83.6, Math.toRadians(180));
    private final Pose throughFirstLine = new Pose(15.4,84.1, Math.toRadians(180));
    private final Pose secondLine = new Pose(43.0,56.4, Math.toRadians(180));
    private final Pose throughSecondLine = new Pose(26.0,56.3, Math.toRadians(180));
    private final Pose readyTele = new Pose(28.5,70.9, Math.toRadians(90));

    private PathChain p1, p2, p3, p4, p5, p6, p7, p8;

    /* ---------------- Init ---------------- */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

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

        buildPaths();
        follower.setPose(startPose);

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
    }

    /* ---------------- Paths ---------------- */
    private void buildPaths() {
        p1 = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose)).build();
        p2 = follower.pathBuilder().addPath(new BezierLine(shootPose, firstLine)).build();
        p3 = follower.pathBuilder().addPath(new BezierLine(firstLine, throughFirstLine)).build();
        p4 = follower.pathBuilder().addPath(new BezierLine(throughFirstLine, shootPose)).build();
        p5 = follower.pathBuilder().addPath(new BezierLine(shootPose, secondLine)).build();
        p6 = follower.pathBuilder().addPath(new BezierLine(secondLine, throughSecondLine)).build();
        p7 = follower.pathBuilder().addPath(new BezierLine(throughSecondLine, shootPose)).build();
        p8 = follower.pathBuilder().addPath(new BezierLine(shootPose, readyTele)).build();
    }

    /* ---------------- Shooting Logic ---------------- */
    private void runShooter(PathState nextState) {
        boolean detected = !laserInput.getState(); // ACTIVE-LOW

        outtake1.setVelocity(TARGET_VELOCITY);
        outtake2.setVelocity(TARGET_VELOCITY);

        double avgVel = (outtake1.getVelocity() + outtake2.getVelocity()) / 2.0;
        boolean atSpeed = Math.abs(avgVel - TARGET_VELOCITY) <= VELOCITY_TOLERANCE;

        if (atSpeed) {
            intake.setPower(1);
            indexer1.setPower(1);
            indexer2.setPower(1);
        } else {
            intake.setPower(0);
            indexer1.setPower(0);
            indexer2.setPower(0);
        }

        // Count artifact leaving sensor
        if (lastDetected && !detected) {
            shotsThisState++;
        }
        lastDetected = detected;

        if (shotsThisState >= 1) {
            stopAll();
            setPathState(nextState);
        }

        telemetry.addData("Detected", detected);
        telemetry.addData("Shots", shotsThisState);
    }

    private void stopAll() {
        outtake1.setVelocity(0);
        outtake2.setVelocity(0);
        intake.setPower(0);
        indexer1.setPower(0);
        indexer2.setPower(0);
    }

    /* ---------------- Intake Mode ---------------- */
    private void runAutoIntake() {
        boolean detected = !laserInput.getState();
        intake.setPower(1);
        indexer1.setPower(1);
        indexer2.setPower(detected ? 0 : 1);
    }

    /* ---------------- State Machine ---------------- */
    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(p1, true);
                if (!follower.isBusy()) setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                runShooter(PathState.FIRST_LINE);
                break;

            case FIRST_LINE:
                follower.followPath(p2, true);
                if (!follower.isBusy()) setPathState(PathState.THROUGH_FIRST_LINE);
                break;

            case THROUGH_FIRST_LINE:
                runAutoIntake();
                follower.followPath(p3, true);
                if (!follower.isBusy()) setPathState(PathState.BACK_TO_SHOOT1);
                break;

            case BACK_TO_SHOOT1:
                follower.followPath(p4, true);
                if (!follower.isBusy()) setPathState(PathState.SHOOT_FIRST);
                break;

            case SHOOT_FIRST:
                runShooter(PathState.SECOND_LINE);
                break;

            case SECOND_LINE:
                follower.followPath(p5, true);
                if (!follower.isBusy()) setPathState(PathState.THROUGH_SECOND_LINE);
                break;

            case THROUGH_SECOND_LINE:
                runAutoIntake();
                follower.followPath(p6, true);
                if (!follower.isBusy()) setPathState(PathState.BACK_TO_SHOOT2);
                break;

            case BACK_TO_SHOOT2:
                follower.followPath(p7, true);
                if (!follower.isBusy()) setPathState(PathState.SHOOT_SECOND);
                break;

            case SHOOT_SECOND:
                runShooter(PathState.READY_TELE);
                break;

            case READY_TELE:
                follower.followPath(p8, true);
                break;
        }

        telemetry.addData("State", pathState);
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsThisState = 0;
        lastDetected = false;
    }
}
