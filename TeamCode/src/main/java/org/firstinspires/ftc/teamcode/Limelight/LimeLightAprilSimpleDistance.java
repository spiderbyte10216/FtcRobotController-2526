package org.firstinspires.ftc.teamcode.Limelight;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Limelight.TestBench;

@TeleOp
public class LimeLightAprilSimpleDistance extends OpMode{
    private Limelight3A limelight3A;
    TestBench bench = new TestBench();
    private double distance;

    @Override
    public void init() {
        bench.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
        //get yaw from control hub IMU
        YawPitchRollAngles orientation = bench.getOrientation();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        //get latest limelight result, pipeline 8 for April tag 20
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            distance = getDistanceFromTage(llResult.getTa());
            telemetry.addData("Calculated Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botpose.toString());

        } else {
            telemetry.addLine("No valid result");
        }
        telemetry.update();
    }

    public double getDistanceFromTage(double ta) {
        double scale = 1871.01567;
        double distance = Math.pow((ta/scale), (1/-1.44028));
        return distance;

    }

}
