package org.firstinspires.ftc.teamcode.parts;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Mike on 11/17/2016.
 */

public class RgbSensor {

    public ColorSensor sensorRGB;
    private DeviceInterfaceModule cdim;
    private View relativeLayout = null;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 5;

    public float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    private float values[];

    public RgbSensor(HardwareMap hardwareMap, String sensorName, String ledName) {
        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get(ledName);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get(sensorName);

        init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap) {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        values = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        ledOn();
    }

    public void ledOn() {
        cdim.setDigitalChannelState(LED_CHANNEL, true);
    }

    public void ledOff() {
        cdim.setDigitalChannelState(LED_CHANNEL, false);
    }

    public void run() {

        // convert the RGB values to HSV values.
        Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
    }

}
