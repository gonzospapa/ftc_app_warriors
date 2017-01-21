package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Mike on 12/22/2016.
 */
@TeleOp(name="WarriorsBot: Teleop Tank M Test", group="WarriorsBot")
//@Disabled
public class MotorTest extends OpMode {

    public DcMotor motor  = null;
    public DcMotor motorRight  = null;
    private double power = 0;
    private long currentTime = 0;
    private int currentTicks = 0;
    private int currentTicksLAndRMotors[] = {0,0};

    OpticalDistanceSensor odsSensor;  // Hardware Device Object
    double LeftRightMotosPower[] = {0.0, 0.0};

    @Override
    public void init() {
        motor   = hardwareMap.dcMotor.get("left_ball");
        motorRight   = hardwareMap.dcMotor.get("right_ball");

        initMotor(motor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        initMotor(motorRight, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Warrior");
        updateTelemetry(telemetry);

        currentTime = System.currentTimeMillis();



    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            ControlCrossBowMotorsSpeed(true);
        }

        if (gamepad1.b) {
            ControlCrossBowMotorsSpeed(false);
        }



        //motor.setPower(speed);


        updateTelemetry(telemetry);

    }
    public double getSpeedtoPower(double power){ return 864.31*power+3228.81; }

    public double getPowertoSpeed(double speed){return (speed-3228.81)/864.31;}

    private void initMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode runMode, double power) {
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setPower(power);
    }

    public double calcElapsedTime() {
        double elapsedTime = 0;
        long now = System.currentTimeMillis();
        elapsedTime = (now - currentTime) / 1000.0;
        currentTime = now;
        return elapsedTime;
    }

    public boolean isTime() {
        double elapsedTime = 0;
        long now = System.currentTimeMillis();
        elapsedTime = (now - currentTime);
        return elapsedTime >=500;
    }

    public void ramUpMotor(DcMotor motor) {
        setUpBallMotorSpeed(0.0, motor);
        setUpBallMotorSpeed(0.05, motor);
        setUpBallMotorSpeed(0.10, motor);
        setUpBallMotorSpeed(0.15, motor);
        setUpBallMotorSpeed(0.20, motor);
        setUpBallMotorSpeed(0.25, motor);
        setUpBallMotorSpeed(0.30, motor);
        setUpBallMotorSpeed(0.35, motor);
        setUpBallMotorSpeed(0.40, motor);
    }


    public void ramDownMotor(DcMotor motor) {
        setUpBallMotorSpeed(0.35, motor);
        setUpBallMotorSpeed(0.30, motor);
        setUpBallMotorSpeed(0.25, motor);
        setUpBallMotorSpeed(0.20, motor);
        setUpBallMotorSpeed(0.15, motor);
        setUpBallMotorSpeed(0.10, motor);
        setUpBallMotorSpeed(0.05, motor);
        setUpBallMotorSpeed(0.0, motor);
    }
    //sets the ball motor speed with a slight delay.
    public void setUpBallMotorSpeed(double speed, DcMotor motor) {
        Utils.delay(1000);
        motor.setPower(speed);
    }
    public double ConvertSharpSensorvToInches(double volts) {

        double distance = 11.1273 * (1.0/volts);

        return Math.pow(distance,1.996);
    }

    public void ControlCrossBowMotorsSpeed(boolean ramp_up)
    {
        double PowerIncrease = 0.05;
        double TargetSpeed = 3000;
        double AllowedError = 50;
        if (isTime()) {
            double elapsedTimeSec = calcElapsedTime();
            int currentPos[] = {motor.getCurrentPosition(),motorRight.getCurrentPosition()};
            int elapsedTicks[] = {currentPos[0] - currentTicksLAndRMotors[0],currentPos[1] - currentTicksLAndRMotors[1]};


            currentTicksLAndRMotors[0] = currentPos[0];
            currentTicksLAndRMotors[1] = currentPos[1];

            double ticksPerSec[] = {0,0};
            if (elapsedTicks[0] != 0 && elapsedTimeSec != 0) {
                ticksPerSec[0] = elapsedTicks[0] / elapsedTimeSec;
                ticksPerSec[1] = elapsedTicks[1] / elapsedTimeSec;
            }

            telemetry.addData("motor:", motor.getPower());
            telemetry.addData("motorRight:", motorRight.getPower());
            telemetry.addData("ticks/sec:", String.valueOf(ticksPerSec[0]) + "");
            telemetry.addData("ticks/secRight:", String.valueOf(ticksPerSec[1]) + "");
            telemetry.addData("data:", String.valueOf(elapsedTicks[0]) + " - " + String.valueOf(elapsedTimeSec));
            telemetry.addData("data:", String.valueOf(elapsedTicks[0]) + " - " + String.valueOf(elapsedTimeSec));
            //power = getPowertoSpeed(ticksPerSec);

            if (ramp_up)
            {
                if (motor.getPower() < .60) {
                    motor.setPower(Range.clip(motor.getPower()+PowerIncrease,0,1));
                    motorRight.setPower(Range.clip(motorRight.getPower()+PowerIncrease,0,1));
                }
                else
                {
                    if (ticksPerSec[0] < (TargetSpeed-AllowedError))
                    {
                        motor.setPower(motor.getPower()+(PowerIncrease/10.0));
                    }

                    if (ticksPerSec[0] > (TargetSpeed+AllowedError))
                    {
                        motor.setPower(motor.getPower()-(PowerIncrease/10.0));
                    }

                    if (ticksPerSec[1] < (TargetSpeed-AllowedError))
                    {
                        motorRight.setPower(motorRight.getPower()+PowerIncrease/10.0);
                    }
                    if (ticksPerSec[1] > (TargetSpeed+AllowedError))
                    {
                        motorRight.setPower(motorRight.getPower()-(PowerIncrease/10.0));
                    }

                }

            }
            else {
                if (motor.getPower() > 0.0)
                {
                    motor.setPower(Range.clip(motor.getPower()-PowerIncrease,0.0,1));
                    motorRight.setPower(Range.clip(motorRight.getPower()-PowerIncrease,0.0,1));
                }

            }


        }

    }
}
