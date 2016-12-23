package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Mike on 12/22/2016.
 */
@TeleOp(name="WarriorsBot: Teleop Tank", group="WarriorsBot")
public class MotorTest extends OpMode {

    public DcMotor motor  = null;
    private double speed = 0;
    private long currentTime = 0;
    private int currentTicks = 0;
    @Override
    public void init() {
        motor   = hardwareMap.dcMotor.get("motor");

        initMotor(motor, DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Warrior");
        updateTelemetry(telemetry);

        currentTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            speed = speed + .05;
        }

        if (gamepad1.b) {
            speed = speed - .05;
        }

        motor.setPower(speed);

        double elapsedTimeSec = calcElapsedTime();
        int elapsedTicks = motor.getCurrentPosition() - currentTicks;
        double ticksPerSec = 0;
        if (elapsedTicks != 0 && elapsedTimeSec != 0) {
            ticksPerSec = elapsedTicks / elapsedTimeSec;
        }

        telemetry.addData("motor:", speed);
        telemetry.addData("ticks/sec:", String.valueOf(ticksPerSec) + "");
        updateTelemetry(telemetry);
    }

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

}
