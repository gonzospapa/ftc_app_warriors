package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.parts.Imu;
import org.firstinspires.ftc.teamcode.parts.RgbSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class  WiredHardware
{
    /* Public OpMode members. */
    public DcMotor leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor leftBackMotor   = null;
    public DcMotor  rightBackMotor  = null;
    public DcMotor  leftBallmotor    = null;
    public DcMotor  rightBallmotor    = null;
    public DcMotor elevatorMotor = null;
    public DcMotor sweeperMotor = null;

    public Imu brains;

    public OpticalDistanceSensor bottomColorSensor;

    //public RgbSensor colorSensorFront = null;
    //public RgbSensor colorSensorDownLeft = null;
   // public RgbSensor colorSensorDownRight = null;

    public Servo buttonPusher = null;

    public static final double MID_SERVO       =  0.5;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    private Telemetry telemetry;

    /* Constructor */
    public WiredHardware(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /* Initialize standard Hardware interfaces */
    public BotMotion init(HardwareMap ahwMap) {

        BotMotion botMotion = new BotMotion();

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define the hardware.
        mapHardwareDevices(hwMap);

        // Setup the motors
        initMotors(botMotion);

        // Define and initialize ALL installed servos.
        buttonPusher.setPosition(MID_SERVO);
        buttonPusher.setDirection(Servo.Direction.FORWARD);

        //brains.init(ahwMap);

        return botMotion;
    }

    //Map the phone configuration to the code.
    private void mapHardwareDevices(HardwareMap ahwMap) {
        leftMotor   = ahwMap.dcMotor.get("left_front");
        rightMotor  = ahwMap.dcMotor.get("right_front");

        leftBackMotor = ahwMap.dcMotor.get("left_back");
        rightBackMotor = ahwMap.dcMotor.get("right_back");

        leftBallmotor    = ahwMap.dcMotor.get("left_ball");
        rightBallmotor    = ahwMap.dcMotor.get("right_ball");

        elevatorMotor = ahwMap.dcMotor.get("elevator");
        sweeperMotor = ahwMap.dcMotor.get("sweep");

        bottomColorSensor = ahwMap.opticalDistanceSensor.get("btm_color_sensor");

        //colorSensorFront = new RgbSensor(ahwMap, "color_front", "dim");
        //colorSensorDownLeft = new RgbSensor(ahwMap, "color_left", "dim");
        //colorSensorDownRight = new RgbSensor(ahwMap, "color_right", "dim");

        buttonPusher = ahwMap.servo.get("pusher");
    }

    //sets up a single motor
    private void initMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode runMode, double startPower) {
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setPower(startPower);
    }

    //Sets up all of the motors.
    private void initMotors(BotMotion botMotion) {

        initMotor(leftMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        //initMotor(leftBackMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.STOP_AND_RESET_ENCODER, 0);
        initMotor(leftBackMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(leftBallmotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(rightBallmotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(rightMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        //initMotor(rightBackMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.STOP_AND_RESET_ENCODER, 0);
        initMotor(rightBackMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(elevatorMotor, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(sweeperMotor, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);

        telemetry.addData("Say", "initMotors set");

        // Set all motors to zero power
        setAllMotors(botMotion);
    }

    //starts and runs the elevator and sweeper
    public void startupElevatorMotor(BotMotion botMotion) {
        if (botMotion.isElevatorOn && botMotion.newElevatorSpeed == 0) {
            setUpElevatorMotorSpeed(0.06  , botMotion);
            setUpElevatorMotorSpeed(0.125, botMotion);
          //  setUpSweeperMotorSpeed(0.13, botMotion);
          //  setUpSweeperMotorSpeed(0.25, botMotion);
        } else if (botMotion.isElevatorOn == false) {
            setUpElevatorMotorSpeed(0.0, botMotion);
           // setUpSweeperMotorSpeed(0.0, botMotion);
        } //else if (botMotion.isSweepRev == 1 && sweeperMotor.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
          //  sweeperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
          //  setUpSweeperMotorSpeed(0.25, botMotion);
        //} else if (botMotion.isSweepRev == 2 && sweeperMotor.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
           // sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
          //  setUpSweeperMotorSpeed(0.25, botMotion);
        //}
    }

    //resets the elevator speed with a slight delay.
    public void setUpElevatorMotorSpeed(double speed, BotMotion botMotion) {
    Utils.delay(100);

    botMotion.newElevatorSpeed = speed;
    this.elevatorMotor.setPower(botMotion.newElevatorSpeed);
}
    //resets the sweeper speed with a slight delay.
    public void setUpSweeperMotorSpeed(double speed, BotMotion botMotion) {
        Utils.delay(100);

        botMotion.newSweeperSpeed = speed;
        this.sweeperMotor.setPower(botMotion.newSweeperSpeed);
    }

    //sets up the ball motors.
    public void startUpBallMotor(BotMotion botMotion) {
        if (botMotion.isBallMotorOn && botMotion.newBallMotorSpeed == 0) {
            setUpBallMotorSpeed(0.12, botMotion);
            setUpBallMotorSpeed(0.25, botMotion);
            setUpBallMotorSpeed(0.45, botMotion);
        } else if (botMotion.isBallMotorOn == false) {
            setUpBallMotorSpeed(0.0, botMotion);
        }
    }

    //sets the ball motor speed with a slight delay.
    public void setUpBallMotorSpeed(double speed, BotMotion botMotion) {
        Utils.delay(100);

        botMotion.newBallMotorSpeed = speed;
        this.leftBallmotor.setPower(botMotion.newBallMotorSpeed);
        this.rightBallmotor.setPower(botMotion.newBallMotorSpeed);
    }


    public void setAllMotors(BotMotion botMotion) {
        this.leftMotor.setPower(botMotion.newLeftMotorPower);
        this.leftBackMotor.setPower(botMotion.newLeftMotorPower);
        this.rightMotor.setPower(botMotion.newRightMotorPower);
        this.rightBackMotor.setPower(botMotion.newRightMotorPower);
        this.leftBallmotor.setPower(botMotion.newBallMotorSpeed);
        this.rightBallmotor.setPower(botMotion.newBallMotorSpeed);
        this.sweeperMotor.setPower(botMotion.newElevatorSpeed);
        this.elevatorMotor.setPower(botMotion.newElevatorSpeed);

        telemetry.addData("Say", "Speed set");
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
