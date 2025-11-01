package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;

@TeleOp(name="Color Sensor Purple/Green Detect", group="Sensor")
public class ColorSensorTest extends LinearOpMode {

    RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {
            // Get normalized RGBA values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Determine detected color
            String detectedColor = "Unknown";

            // Emphasis on Purple (higher red+blue, lower green)
            if ((red > 100 && blue > 150) && (green > 150)) {
                detectedColor = "Purple";
            }
            // Emphasis on Green (green much stronger than red+blue)
            else if (green > 100 && green > red + 30 && green > blue + 30) {
                detectedColor = "Green";
            }

            // Output to Driver Hub
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Detected", detectedColor);
            telemetry.update();
        }
    }
}