package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="WarriorsBot: Teleop Tank", group="WarriorsBot")
public class WarriorsTank extends OpMode {
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
            botMotion = robot.init(hardwareMap);

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

        try {

            // Things to try. 1. just make sure it's looping run a try catch.

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this section we are going to pickup what the controllers are doing.
            botMotion.newLeftMotorPower = -gamepad1.left_stick_y;
            botMotion.newRightMotorPower = -gamepad1.right_stick_y;
            
            robot.sweeperMotor.setPower(-gamepad2.left_stick_y);

             if (botMotion.newLeftMotorPower > .75) {
                botMotion.newLeftMotorPower = .75;
            } else if (botMotion.newLeftMotorPower < -.75) {
                botMotion.newLeftMotorPower = -.75;
            }


            if (botMotion.newRightMotorPower > .75) {
                botMotion.newRightMotorPower = .75;
            } else if (botMotion.newRightMotorPower < -.75) {
                botMotion.newRightMotorPower = -.75;
            }

            if (gamepad1.right_bumper) {
                robot.buttonPusher.setPosition(.75);
                robot.buttonPusher.setDirection(Servo.Direction.FORWARD);
            } else if (gamepad1.left_bumper) {
                robot.buttonPusher.setPosition(.75);
                robot.buttonPusher.setDirection(Servo.Direction.REVERSE);
            } else if (gamepad1.a && robot.buttonPusher.getDirection() == Servo.Direction.REVERSE) {
                robot.buttonPusher.setPosition(.45);
                robot.buttonPusher.setDirection(Servo.Direction.FORWARD);
            } else if (gamepad1.a && robot.buttonPusher.getDirection() == Servo.Direction.FORWARD) {
                robot.buttonPusher.setPosition(.45);
                robot.buttonPusher.setDirection(Servo.Direction.REVERSE);
            }

            if (gamepad2.a) {
                botMotion.isBallMotorOn = true;
           } else if (gamepad2.b) {
                botMotion.isBallMotorOn = false;
                botMotion.newBallMotorSpeed = 0;
           }

            if (gamepad2.x) {
                botMotion.isSweepRev = 1;
            } else if (gamepad2.y) {
                botMotion.isSweepRev = 2;
            }

            if (gamepad2.left_bumper) {
                botMotion.isElevatorOn = true;
            } else if (gamepad2.right_bumper) {
                botMotion.isElevatorOn = false;
            }



            robot.setAllMotors(botMotion);

            robot.startUpBallMotor(botMotion);

            robot.startupElevatorMotor(botMotion);

            botMotion.isSweepRev = 0;

            // Send telemetry message to signify robot running;
            telemetry.addData("left", "%.2f", botMotion.newLeftMotorPower);
            telemetry.addData("right", "%.2f", botMotion.newRightMotorPower);

            if (errorMsg != null) {
                telemetry.addData("err", errorMsg);
            } else {
                telemetry.addData("ball", "%.2f", botMotion.newBallMotorSpeed);
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
