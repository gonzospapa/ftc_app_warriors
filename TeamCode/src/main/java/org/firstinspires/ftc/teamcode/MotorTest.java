package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Mike on 12/22/2016.
 */
@TeleOp(name="WarriorsBot: Teleop Tank", group="WarriorsBot")
@Disabled
public class MotorTest extends OpMode {

    public DcMotor motor  = null;
    private double power = 0;
    private long currentTime = 0;
    private int currentTicks = 0;

    OpticalDistanceSensor odsSensor;  // Hardware Device Object

    @Override
    public void init() {
        motor   = hardwareMap.dcMotor.get("left_ball");
        odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");


        initMotor(motor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Warrior");
        updateTelemetry(telemetry);

        currentTime = System.currentTimeMillis();



    }

    @Override
    public void loop() {
        if (gamepad1.a) {
                ramUpMotor(motor);
        }

        if (gamepad1.b) {
                ramDownMotor(motor);
        }

        //motor.setPower(speed);

        power = motor.getPower();

        if (power > 0 && isTime()) {
            double elapsedTimeSec = calcElapsedTime();
            int currentPos = motor.getCurrentPosition();
            int elapsedTicks = currentPos - currentTicks;
            currentTicks = currentPos;
            double ticksPerSec = 0;
            if (elapsedTicks != 0 && elapsedTimeSec != 0) {
                ticksPerSec = elapsedTicks / elapsedTimeSec;
            }

            power = getPowertoSpeed(ticksPerSec);
            if (power > 40) {
                double deltaPower = power - 40;
                motor.setPower(power - deltaPower);
            } else if (power < 40) {
                double deltaPower = 40 - power;
                motor.setPower(power + deltaPower);
            }

            telemetry.addData("motor:", power);
            telemetry.addData("ticks/sec:", String.valueOf(ticksPerSec) + "");
            telemetry.addData("data:", String.valueOf(elapsedTicks) + " - " + String.valueOf(elapsedTimeSec));
            // send the info back to driver station using telemetry function.
            telemetry.addData("Raw",    odsSensor.getRawLightDetected());
            telemetry.addData("Normal", odsSensor.getLightDetected());

        }
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
        return elapsedTime >= 1000;
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

}
