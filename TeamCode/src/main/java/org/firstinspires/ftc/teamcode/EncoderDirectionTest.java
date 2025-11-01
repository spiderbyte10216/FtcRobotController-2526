package org.firstinspires.ftc.teamcode; // <- keep/adjust this to match your package

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="EncoderDirectionTest", group="Debug")
public class EncoderDirectionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // inPerTick can be anything nonzero for this test, we're just reading raw ticks
        ThreeDeadWheelLocalizer localizer =
                new ThreeDeadWheelLocalizer(hardwareMap, 0.001, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            // read each tracking wheel encoder
            PositionVelocityPair par0 = localizer.par0.getPositionAndVelocity();
            PositionVelocityPair par1 = localizer.par1.getPositionAndVelocity();
            PositionVelocityPair perp = localizer.perp.getPositionAndVelocity();

            telemetry.addData("par0 ticks", par0.position);
            telemetry.addData("par1 ticks", par1.position);
            telemetry.addData("perp ticks", perp.position);
            telemetry.update();
        }
    }
}
