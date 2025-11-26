package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // 1) start FLUSH in the blue bottom-left corner, pointed to the middle
        Pose2d START = new Pose2d(
                -52,                // x
                -52,                // y (negative = bottom)
                Math.toRadians(45)  // facing up-right / toward center
        );

        // 2) mirrored targets (bottom side)
        Vector2d LAUNCH      = new Vector2d(-16, -16);  // “back” along the 45° diagonal
        Vector2d FIRST_LINE  = new Vector2d(-8, -50);  // first run toward the center-ish
        Vector2d SECOND_LINE = new Vector2d(-40, -80);  // a bit farther down (still bottom side)

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(START)   // 👈 makes MM actually draw it in the corner
                .setConstraints(
                        60,
                        60,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        15
                )
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(START)
                        // BACK out of the corner along the white diagonal
                        .splineToConstantHeading(LAUNCH, Math.toRadians(45))

                        // go to the first “line” area (still bottom half)
                        .splineTo(FIRST_LINE, Math.toRadians(45))

                        // return to the launch spot on the diagonal
                        .splineToConstantHeading(LAUNCH, Math.toRadians(225))

                        // square up — on blue bottom side we keep 45°
                        .turnTo(Math.toRadians(45+180))

                        // second cycle
                        .splineTo(SECOND_LINE, Math.toRadians(0))
                        .lineToX(-25)
                        .splineToConstantHeading(LAUNCH, Math.toRadians(45))
                        .turnTo(Math.toRadians(45))
                        .build()
        );

        meepMeep
                .setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
