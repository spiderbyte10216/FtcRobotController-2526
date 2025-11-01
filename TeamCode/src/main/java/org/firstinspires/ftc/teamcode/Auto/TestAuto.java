package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "TestAuto", group = "Robot")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-30, 24, Math.toRadians(315)); // 135+180

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(315)) //goes backward to launch zone
                .splineTo(new Vector2d(-40, 60), Math.toRadians(0)) //goes to first line
                .lineToX(-25) //goes forward to intake artifacts
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(315)) //goes back to launch zone
                .turnTo(Math.toRadians(315)) //rotate to square up to goal
                .splineTo(new Vector2d(-40, 80), Math.toRadians(0)) // goes to second line
                .lineToX(-25) //goes forward to intake artifacts
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(315)) //goes back to launch zone
                .turnTo(Math.toRadians(315)); //rotate to square up to goal



        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(tab.build());
    }
}
