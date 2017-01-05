package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.instructions.BotInstruction;
import org.firstinspires.ftc.teamcode.instructions.Move;
import org.firstinspires.ftc.teamcode.instructions.MoveLineDetect;
import org.firstinspires.ftc.teamcode.instructions.PushButton;
import org.firstinspires.ftc.teamcode.instructions.Start;
import org.firstinspires.ftc.teamcode.instructions.Stop;
import org.firstinspires.ftc.teamcode.instructions.Turn;

/**
 * Created by Mike Murphy on 11/18/2016.
 */
@Autonomous(name = "WarriorsBot: LinearOpMode", group = "WarriorsAuto")
@Disabled                     // Comment this out to add to the opmode list
public class WarriorsAuto extends LinearOpMode {



    /* Declare OpMode members. */
    private WiredHardware robot = new WiredHardware(telemetry);  // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private BotMotion botMotion = null;

    private String errorMsg = null;

    Start start = null;

    @Override
    public void runOpMode() throws InterruptedException {

        BotMotion botMotion = initBot();

        BotInstruction currentInstruction = start;

        while (opModeIsActive())  {

            botMotion.X_Position_Inches = convert_encoder_dat_to_inches((robot.leftBackMotor.getCurrentPosition() + robot.rightBackMotor.getCurrentPosition()) / 2);

            //telemetry.addData("Msg:", "InLoop");

            telemetry.addData("State:", currentInstruction.getName());

            if (currentInstruction.isComplete()) {
                currentInstruction = currentInstruction.transition();
                currentInstruction.start();
            }

            if (currentInstruction == null) {
                break;
            }

            currentInstruction.doWork();

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }

    private BotMotion initBot() {
        BotMotion botMotion = robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Warrior");
        updateTelemetry(telemetry);

        //
        telemetry.addData("Msg:", opModeIsActive());
        telemetry.update();


        //Stop stop = new Stop();
        //Move move6 = new Move("Move6", false, 10L, stop, robot, botMotion);
        //Turn turn3 = new Turn("Turn3", true, 90L, move6, robot, botMotion);
        //Move move5 = new Move ("Move5", true, 20L, turn3, robot, botMotion);
        //PushButton pushbutton2 = new PushButton("PushButton2",move5,robot,botMotion);
        //MoveLineDetect moveLineDetect2 = new MoveLineDetect("Move4",true,20L,pushbutton2,robot,botMotion);
       //PushButton pushbutton1 = new PushButton("PushButton2",moveLineDetect2,robot,botMotion);
        //MoveLineDetect moveLineDetect1 = new MoveLineDetect("Move3", true, 20L, pushbutton1, robot, botMotion);
        //Turn turn2 = new Turn("Turn2", true, 90L, moveLineDetect1, robot, botMotion);
        //Move move2 = new Move("Move2",true,20L,turn2,robot,botMotion);
        //Turn turn1 = new Turn("Turn1",true,20L,move2,robot,botMotion);

        Stop stop = new Stop(robot);
        Move move1 = new Move("Move1",true,10L,stop,robot,botMotion);
        start = new Start(move1);

        return botMotion;
    }

    double convert_encoder_dat_to_inches (double encoderVal)
    {
        return (encoderVal)/117;

    }
}
