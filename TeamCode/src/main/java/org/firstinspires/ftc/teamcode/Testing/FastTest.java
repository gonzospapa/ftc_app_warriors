package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.WiredHardware;

/**
 * Created by Mike on 1/18/2017.
 */

@TeleOp(name="WarriorsBot: Teleop Tank", group="WarriorsBot")
@Disabled
public class FastTest extends OpMode {
    /* Declare OpMode members. */
    private WiredHardware robot = new WiredHardware(telemetry); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private BotMotion botMotion = null;

    private String errorMsg = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        botMotion = robot.init(hardwareMap, false, false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Warrior");
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     * Need to create a kill switch..
     */
    @Override
    public void loop() {


            // Things to try. 1. just make sure it's looping run a try catch.

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this section we are going to pickup what the controllers are doing.
            robot.leftMotor.setPower(-gamepad1.left_stick_y);
            robot.rightMotor.setPower(-gamepad1.right_stick_y);

            updateTelemetry(telemetry);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

