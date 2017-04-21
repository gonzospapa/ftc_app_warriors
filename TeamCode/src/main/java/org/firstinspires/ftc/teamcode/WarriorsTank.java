package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="WarriorsBot: Teleop Rat", group="WarriorsBot")
public class WarriorsTank extends OpMode {
    /* Declare OpMode members. */
    private WiredHardware robot = new WiredHardware(telemetry); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private BotMotion botMotion = null;
    private double leftLeftAjust=0.08;
    private double leftRightAjust=0.5;
    private double rightLeftAjust=0.5;
    private double rightRightAjust=0.08;

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
            telemetry.addData("Say", "Hello Warrior V1");
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

        try {

            // Things to try. 1. just make sure it's looping run a try catch.

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this section we are going to pickup what the controllers are doing.
            botMotion.newLeftMotorPower = -gamepad1.left_stick_y*0.75;
            botMotion.newRightMotorPower = -gamepad1.right_stick_y*0.75;



            if (gamepad1.right_bumper) {
                rightLeftAjust=rightLeftAjust+(-gamepad1.left_stick_y*0.05);
                rightRightAjust=rightRightAjust+(-gamepad1.right_stick_y*0.05);
                botMotion.newLeftMotorPower = rightLeftAjust;
                botMotion.newRightMotorPower = rightRightAjust;// 0.1  0.05
            }
            if (gamepad1.left_bumper) {
                botMotion.newLeftMotorPower = leftLeftAjust;
                botMotion.newRightMotorPower = leftRightAjust;
            }
            robot.setAllMotors(botMotion);
            if (errorMsg != null) {
                telemetry.addData("err", errorMsg);
            }
            updateTelemetry(telemetry);

        } catch (Exception ex) {
            errorMsg = ex.getMessage();
            telemetry.addData("err", errorMsg);
            updateTelemetry(telemetry);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
