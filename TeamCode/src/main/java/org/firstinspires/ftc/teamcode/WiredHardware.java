package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.parts.Imu;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

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

    public BNO055IMU imu;
    public BNO055IMU.Parameters parameters;

    public  OpticalDistanceSensor odsSensor;  // Hardware Device Object
    public  OpticalDistanceSensor odsSensorForLineDetect;  // Hardware Device Object
    public ColorSensor sensorRGB;

    //public OpticalDistanceSensor bottomColorSensor;

    //public RgbSensor colorSensorFront = null;
    //public RgbSensor colorSensorDownLeft = null;
    // public RgbSensor colorSensorDownRight = null;

    public Servo buttonPusher = null;

    public static final double MID_SERVO       =  0.5;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public Telemetry telemetry;

    /* Constructor */
    public WiredHardware(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public BotMotion init(HardwareMap ahwMap) {
        return this.init(ahwMap, true);
    }

    /* Initialize standard Hardware interfaces */
    public BotMotion init(HardwareMap ahwMap, boolean useIMU) {

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

        if (useIMU) {
            initIMU(ahwMap);
            initODS(ahwMap);
            initAdaFruitRGB(ahwMap);
        }

        return botMotion;
    }

    private void initIMU(HardwareMap ahwMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private void initODS(HardwareMap ahwMap) {

        odsSensor = ahwMap.opticalDistanceSensor.get("sensor_ods");

        odsSensorForLineDetect = ahwMap.opticalDistanceSensor.get("sensor_ods_line");
    }
    private void initAdaFruitRGB(HardwareMap ahwMap) {

        // get a reference to our ColorSensor object.
        sensorRGB = ahwMap.colorSensor.get("sensor_color");
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

        buttonPusher = ahwMap.servo.get("pusher");
    }

    //sets up a single motor
    private void initMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode runMode, double startPower) {
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setPower(startPower);
    }

    public void setMotorDirection(boolean foward) {

        if (foward) {
            initMotor(leftMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
            initMotor(leftBackMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
            initMotor(rightMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
            initMotor(rightBackMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        } else {
            initMotor(leftMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
            initMotor(leftBackMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
            initMotor(rightMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
            initMotor(rightBackMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        }

    }

    //Sets up all of the motors.
    private void initMotors(BotMotion botMotion) {

        initMotor(leftMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(leftBackMotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(leftBallmotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(rightBallmotor, DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(rightMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(rightBackMotor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(elevatorMotor, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(sweeperMotor, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);

        // Set all motors to zero power
        setAllMotors(botMotion);
    }

    //starts and runs the elevator and sweeper
    public void startupElevatorMotor(BotMotion botMotion) {
        if (botMotion.isElevatorOn && botMotion.newElevatorSpeed == 0) {
            setUpElevatorMotorSpeed(0.06  , botMotion);
            setUpElevatorMotorSpeed(0.08, botMotion);
            setUpSweeperMotorSpeed(0.08, botMotion);
        } else if (botMotion.isElevatorOn == false) {
            setUpElevatorMotorSpeed(0.0, botMotion);
        }
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
    public void setBallMotor(BotMotion botMotion) {
        if (botMotion.shouldBallMotor) {
            if (botMotion.isBallMotorOn) {
                for (double i = botMotion.newBallMotorSpeed; i < .4; i = i + .05) {
                    setUpBallMotorSpeed(i, botMotion);
                }
            } else {
                for (double i = botMotion.newBallMotorSpeed; i > 0; i = i - .05) {
                    setUpBallMotorSpeed(i, botMotion);
                }
            }
        } else {
            if (botMotion.isBallMotorOn) {
                int leftCurrPos = leftBallmotor.getCurrentPosition();
                int leftElapsedTicks = leftCurrPos - botMotion.leftBallMotorTicks;
                botMotion.leftBallMotorTicks = leftCurrPos;
                double leftTicksPerSec = Utils.getTicksPerSecond(leftElapsedTicks, botMotion.elapsedTime);

                int rightCurrPos = rightBallmotor.getCurrentPosition();
                int rightElapsedTicks = rightCurrPos - botMotion.rightBallMotorTicks;
                botMotion.rightBallMotorTicks = rightCurrPos;
                double rightTicksPerSec = Utils.getTicksPerSecond(rightElapsedTicks, botMotion.elapsedTime);
            }
        }
    }

    //sets the ball motor speed with a slight delay.
    public void setUpBallMotorSpeed(double speed, BotMotion botMotion) {
        Utils.delay(1000);

        botMotion.newBallMotorSpeed = speed;
        this.leftBallmotor.setPower(botMotion.newBallMotorSpeed);
        this.rightBallmotor.setPower(botMotion.newBallMotorSpeed);
    }


    public void setAllMotors(BotMotion botMotion) {
        this.leftMotor.setPower(botMotion.newLeftMotorPower);
        this.leftBackMotor.setPower(botMotion.newLeftMotorPower);
        this.rightMotor.setPower(botMotion.newRightMotorPower);
        this.rightBackMotor.setPower(botMotion.newRightMotorPower);
    }

    public void resetEncoders() throws InterruptedException {
        this.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopAllMovement() {
        this.leftMotor.setPower(0.0);
        this.leftBackMotor.setPower(0.0);
        this.rightMotor.setPower(0.0);
        this.rightBackMotor.setPower(0.0);
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

    public double getSpeedtoPower(double power){ return 864.31*power+3228.81; }

    public double getPowertoSpeed(double speed){return (speed-3228.81)/864.31;}

    public double[] getAcclerations() {
        Acceleration acc;
        acc  = this.imu.getLinearAcceleration();
        return new double[]{acc.xAccel, acc.yAccel, acc.zAccel};
    }


    /**
     * This method returns a 3x1 array of doubles with the yaw, pitch, and roll in that order.
     * The equations used in this method came from:
     * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_from_Quaternion
     */
    public double[] getAngles() {
        Quaternion quatAngles = this.imu.getQuaternionOrientation();

        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        // for the Adafruit IMU, yaw and roll are switched
        double roll = Math.atan2( 2*(w*x + y*z) , 1 - 2*(x*x + y*y) ) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) ) * 180.0 / Math.PI;

        return new double[]{yaw, pitch, roll};
    }


    public double calcElapsedTime(BotMotion botMotion) {
        double elapsedTime = 0;
        long now = System.currentTimeMillis();
        elapsedTime = (now - botMotion.currentTimeBallControl) / 1000.0;
        botMotion.currentTimeBallControl = now;
        return elapsedTime;
    }

    public boolean isTime(BotMotion botMotion) {
        double elapsedTime = 0;
        long now = System.currentTimeMillis();
        elapsedTime = (now - botMotion.currentTimeBallControl);
        return elapsedTime >=500;
    }

    public void ControlCrossBowMotorsSpeed(boolean ramp_up, WiredHardware robot, BotMotion botMotion)
    {
        double PowerIncrease = 0.05;
        double TargetSpeed = 8000;
        double AllowedError = 50;
        if (isTime(botMotion)) {
            double elapsedTimeSec = calcElapsedTime(botMotion);
            int currentPos[] = {robot.leftBallmotor.getCurrentPosition(),robot.rightBallmotor.getCurrentPosition()};
            int elapsedTicks[] = {currentPos[0] - botMotion.currentTicksLAndRMotors[0],currentPos[1] - botMotion.currentTicksLAndRMotors[1]};


            botMotion.currentTicksLAndRMotors[0] = currentPos[0];
            botMotion.currentTicksLAndRMotors[1] = currentPos[1];

            double ticksPerSec[] = {0,0};
            if (elapsedTicks[0] != 0 && elapsedTimeSec != 0) {
                ticksPerSec[0] = elapsedTicks[0] / elapsedTimeSec;
                ticksPerSec[1] = elapsedTicks[1] / elapsedTimeSec;
            }



            if (ramp_up)
            {
                if (robot.leftBallmotor.getPower() < 0.8) {
                    robot.leftBallmotor.setPower(Range.clip(robot.leftBallmotor.getPower()+PowerIncrease,0,1));
                    robot.rightBallmotor.setPower(Range.clip(robot.rightBallmotor.getPower()+PowerIncrease,0,1));
                }
                else
                {
                    if (ticksPerSec[0] < (TargetSpeed-AllowedError))
                    {
                        robot.leftBallmotor.setPower(robot.leftBallmotor.getPower()+(PowerIncrease/10.0));
                    }

                    if (ticksPerSec[0] > (TargetSpeed+AllowedError))
                    {
                        robot.leftBallmotor.setPower(robot.leftBallmotor.getPower()-(PowerIncrease/10.0));
                    }

                    if (ticksPerSec[1] < (TargetSpeed-AllowedError))
                    {
                        robot.rightBallmotor.setPower(robot.rightBallmotor.getPower()+PowerIncrease/10.0);
                    }
                    if (ticksPerSec[1] > (TargetSpeed+AllowedError))
                    {
                        robot.rightBallmotor.setPower(robot.rightBallmotor.getPower()-(PowerIncrease/10.0));
                    }

                }

            }
            else {
                if (robot.leftBallmotor.getPower() > 0.0)
                {
                    robot.leftBallmotor.setPower(Range.clip(robot.leftBallmotor.getPower()-PowerIncrease,0.0,1));
                    robot.rightBallmotor.setPower(Range.clip(robot.rightBallmotor.getPower()-PowerIncrease,0.0,1));
                }

            }


        }

    }
}
