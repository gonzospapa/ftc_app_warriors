package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.instructions.BotInstruction;
import org.firstinspires.ftc.teamcode.instructions.Move;
import org.firstinspires.ftc.teamcode.instructions.Start;
import org.firstinspires.ftc.teamcode.instructions.Stop;
import org.firstinspires.ftc.teamcode.instructions.Turn;

/**
 * Created by Mike Murphy on 11/18/2016.
 */
@Autonomous(name = "WarriorsBot: LinearOpMode", group = "WarriorsAuto")
@Disabled                       // Comment this out to add to the opmode list
public class WarriorsAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private WiredHardware robot = new WiredHardware(telemetry);  // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private BotMotion botMotion = null;

    private String errorMsg = null;

    @Override
    public void runOpMode() throws InterruptedException {

        BotMotion botMotion = robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Warrior");
        updateTelemetry(telemetry);

        //
        telemetry.addData("Msg:", opModeIsActive());
        telemetry.update();

        while (opModeIsActive())  {

            telemetry.addData("Msg:", "InLoop");

            botMotion.isBallMotorOn = true;
            robot.startUpBallMotor(botMotion);

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }
}
