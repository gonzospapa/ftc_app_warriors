package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="FTC_Stool Drive: Barstool", group="WarriorsBot")
public class FTC_Rats_Stool_Drive extends OpMode {
    /* Declare OpMode members. */
    private WiredHardware robot = new WiredHardware(telemetry); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private BotMotion botMotion = null;

    private String errorMsg = null;

    private Utils.Vector testvec;

    private Utils.Holonomic_Wheels_PWR TestW;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
            botMotion = robot.init(hardwareMap, false, true);

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
            //botMotion.newLeftMotorPower = -gamepad1.left_stick_y;
            //botMotion.newRightMotorPower = -gamepad1.right_stick_y;

            testvec = Utils.getPolarJoy(-gamepad1.left_stick_x,-gamepad1.left_stick_y);

            TestW = Utils.HolonomicRadianOutput(testvec.radians, testvec.speed,0);
            robot.leftMotor.setPower(TestW.frontLeftOutput);
            robot.rightMotor.setPower(TestW.frontRightOutput);
            robot.leftBackMotor.setPower(TestW.rearLeftOutput);
            robot.rightBackMotor.setPower(TestW.rearRightOutput);
            //botMotion.newRightMotorPower = -gamepad1.right_stick_y;

            // Send telemetry message to signify robot running;
            telemetry.addData("testvec.radians", String.valueOf(testvec.radians) + "");
            telemetry.addData("testvec.speed", String.valueOf(testvec.speed) + "");

            telemetry.addData("TestW.frontLeftOutput", String.valueOf(TestW.frontLeftOutput) + "");
            telemetry.addData("TestW.frontRightOutput", String.valueOf(TestW.frontRightOutput) + "");
            telemetry.addData("TestW.rearLeftOutput", String.valueOf(TestW.rearLeftOutput) + "");
            telemetry.addData("TestW.rearRightOutput", String.valueOf(TestW.rearRightOutput) + "");



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
