package org.firstinspires.ftc.teamcode.OldProgram;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.RgbSensor;

/**
 * Created by Mike on 11/30/2016.
 */
@TeleOp(name="RgbTest", group="RgbTest Examples")
@Disabled
public class RgbTest extends LinearOpMode {

    private RgbSensor rgbSensor = null;
    private String errorMsg;

    @Override
    public void runOpMode() throws InterruptedException {

        rgbSensor = new RgbSensor(hardwareMap,"RgbRight","LedRight");

        while (opModeIsActive())  {

            try {

                rgbSensor.init(hardwareMap);
                rgbSensor.run();

                // get RGB values and dispay them
                telemetry.addData("red", rgbSensor.sensorRGB.red());
                telemetry.addData("blue", rgbSensor.sensorRGB.blue());
                telemetry.addData("green", rgbSensor.sensorRGB.green());

                updateTelemetry(telemetry);

            } catch (Exception ex) {
                errorMsg = ex.getMessage();
                telemetry.addData("err", errorMsg);
                updateTelemetry(telemetry);
            }

        }

    }


}
