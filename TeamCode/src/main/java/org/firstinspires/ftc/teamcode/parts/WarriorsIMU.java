package org.firstinspires.ftc.teamcode.parts;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Mike on 1/14/2017.
 */

public class WarriorsIMU {

    private final BNO055IMU imu;
    private final String name;

    public WarriorsIMU(String name, HardwareMap hwmap) {
        this.name = name;
        imu = hwmap.get(BNO055IMU.class, name);
        setParameters();
    }

    private void setParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    public BNO055IMU getImu() {
        return imu;
    }
}
