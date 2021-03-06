package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.WiredHardware;
import org.firstinspires.ftc.teamcode.instructions.BotInstruction;
import org.firstinspires.ftc.teamcode.instructions.Move;
import org.firstinspires.ftc.teamcode.instructions.Start;
import org.firstinspires.ftc.teamcode.instructions.Stop;
import org.firstinspires.ftc.teamcode.instructions.Turn;
/**
 * Created by Mike Murphy on 11/18/2016.
 */
@Autonomous(name = "SensorTest: LinearOpMode", group = "WarriorsAuto")
@Disabled                     // Comment this out to add to the opmode list
public class SensorTest extends LinearOpMode {



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

            //telemetry.addData("Status", "in Loop");
            //telemetry.update();

            Utils.setTime(botMotion);

            //telemetry.addData("Status", "setTime");
            //telemetry.update();

            botMotion.xAcceleration= robot.getAcclerations()[0]; //

            //telemetry.addData("Status", "set acc.");
            //telemetry.update();

            Utils.IntegrateAcceleration(botMotion);// updates integration to get velocity

            // telemetry.addData("Status", "set vel");
            //telemetry.update();

            // Left Encoder is shot. Ignore it for now
            //botMotion.X_Position_Inches = convert_encoder_dat_to_inches((robot.leftBackMotor.getCurrentPosition() + robot.rightBackMotor.getCurrentPosition()) / 2);
            botMotion.X_Position_Inches = convert_encoder_dat_to_inches( robot.rightBackMotor.getCurrentPosition());

            //telemetry.addData("Status", "set inches");
            //telemetry.update();

            double fixedangle = Utils.convertheading(robot.getAngles()[0]);

            //telemetry.addData("Status", "fixed angle");
            //telemetry.update();

            botMotion.normalizedHeading = Utils.ApplyAngleOffset(fixedangle, botMotion.YawAngleOffset);

            // telemetry.addData("Status", "norm heading");
            //telemetry.update();

            telemetry.addData("accelerationsX", botMotion.xAcceleration);
            telemetry.addData("normalizedHeading", botMotion.normalizedHeading);
            telemetry.addData("headingError",botMotion.headingError);

            telemetry.addData("turningPowercopy", botMotion.turningPowercopy);

            telemetry.addData("xVelocity", botMotion.xVelocity);
            telemetry.addData("Encoder Left", robot.leftBackMotor.getPower());
            telemetry.addData("Encoder Right", robot.rightBackMotor.getPower());
            telemetry.addData("Encoder Left", robot.leftMotor.getPower());
            telemetry.addData("IsWhiteLineThere", Utils.IsWhiteLineThere(this.robot));

            telemetry.addData("SensorReading", robot.odsSensorForLineDetect.getRawLightDetected());

            telemetry.addData("IsBlue", Utils.IsBlue(this.robot));

            telemetry.addData("State:", currentInstruction.getName());




            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }

    private BotMotion initBot() {
        BotMotion botMotion = robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Warrior V1");
        updateTelemetry(telemetry);

        //
        telemetry.addData("Msg:", opModeIsActive());
        telemetry.update();

        // wait to see this on the Driver Station before pressing play, to make sure the IMU has been initialized
        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
        }

        waitForStart();
        telemetry.clear();

        botMotion.targetHeading = 10.0;
        botMotion.YawAngleOffset = Utils.convertheading(robot.getAngles()[0]); // Yaw is first index


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

        Turn turn1 = new Turn("Turn1", false, 90L, stop, robot, botMotion);
        Move move1 = new Move("Move1",true,30L,turn1, robot, botMotion);


        telemetry.clear();
        telemetry.addData("Status", "2");
        telemetry.update();

        start = new Start(move1);

        return botMotion;
    }

    double convert_encoder_dat_to_inches (double encoderVal)
    {
        return (encoderVal)/117;

    }
}