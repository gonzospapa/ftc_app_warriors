package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Mike on 10/11/2016.
 */

public class Imu {

    public BNO055IMU imu;
    public BNO055IMU.Parameters   parameters = new BNO055IMU.Parameters();

    public void init(HardwareMap hardwareMap) throws Exception {
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = ",BNO055";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

}
