package org.firstinspires.ftc.teamcode.OldProgram;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.WiredHardware;

@TeleOp(name="WarriorsBot: Teleop Tank", group="WarriorsBot")
@Disabled
public class WarriorsTankTwo extends LinearOpMode {
    /* Declare OpMode members. */
    private WiredHardware robot = new WiredHardware(telemetry); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private BotMotion botMotion = null;

    private String errorMsg = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void initWarrior() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //try {
            botMotion = robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "Hello Warrior");
            updateTelemetry(telemetry);

        //} catch (Exception ex) {
        //    telemetry.addData("Init Err:", ex.getMessage());
        //    updateTelemetry(telemetry);
        //}


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     * Need to create a kill switch..
     */
    @Override
    public void runOpMode() throws InterruptedException {

            initWarrior();

            while (opModeIsActive()) {
                try {
                // Things to try. 1. just make sure it's looping run a try catch.

                // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
                // In this section we are going to pickup what the controllers are doing.
                botMotion.newLeftMotorPower = -gamepad1.left_stick_y;
                botMotion.newRightMotorPower = -gamepad1.right_stick_y;

                if (gamepad1.right_bumper) {
                    robot.buttonPusher.setPosition(1.0);
                    robot.buttonPusher.setDirection(Servo.Direction.FORWARD);
                } else if (gamepad1.left_bumper) {
                    robot.buttonPusher.setPosition(1.0);
                    robot.buttonPusher.setDirection(Servo.Direction.REVERSE);
                }

                if (gamepad2.a) {
                    botMotion.isBallMotorOn = true;
                } else if (gamepad2.b) {
                    botMotion.isBallMotorOn = false;
                    botMotion.newBallMotorSpeed = 0;
                }

                if (gamepad2.left_bumper) {
                    botMotion.isElevatorOn = true;
                } else if (gamepad2.right_bumper) {
                    botMotion.isElevatorOn = false;
                }

                robot.setAllMotors(botMotion);

                robot.startUpBallMotor(botMotion);

                robot.startupElevatorMotor(botMotion);

                // Send telemetry message to signify robot running;
                telemetry.addData("left", "%.2f", botMotion.newLeftMotorPower);
                telemetry.addData("right", "%.2f", botMotion.newRightMotorPower);

                if (errorMsg != null) {
                    telemetry.addData("err", errorMsg);
                } else {
                    telemetry.addData("ball", "%.2f", botMotion.newBallMotorSpeed);
                }
                updateTelemetry(telemetry);

            }catch(Exception ex){
                errorMsg = ex.getMessage();
                telemetry.addData("err", errorMsg);
                updateTelemetry(telemetry);
            }
        }
    }
}
