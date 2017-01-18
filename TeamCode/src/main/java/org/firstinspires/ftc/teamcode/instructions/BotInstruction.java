package org.firstinspires.ftc.teamcode.instructions;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.WiredHardware;

/**
 * Created by Mike on 11/12/2016.
 */

public abstract class BotInstruction {

    private String name;
    protected BotInstruction nextState;
    protected WiredHardware robot;
    protected BotMotion botMotion;

    public BotInstruction(String name, BotInstruction nextState, WiredHardware robot, BotMotion botMotion) {
        this.name = name;
        this.nextState = nextState;
        this.robot = robot;
        this.botMotion = botMotion;
    }

    public void initHeading() {
        botMotion.angles = this.robot.imu.getAngularOrientation();
        botMotion.normalizedHeading = Utils.normalizePitchReading(Utils.normalizeDegrees(botMotion.angles.angleUnit.fromDegrees(botMotion.angles.firstAngle)), botMotion.initialPitchValue);
        botMotion.X_Position_Inches = Utils.convertEncoderDatToInches((robot.leftBackMotor.getCurrentPosition() + robot.rightBackMotor.getCurrentPosition()) / 2);
        //botMotion.Side_ods_Distance = Utils.get_ods_side_value_in_inches();
        //botMotion.front_ods_Distance = Utils.get_ods_value_in_inches();
    }

    protected void stopRobot() {
        botMotion.newLeftMotorPower = 0.0;
        botMotion.newRightMotorPower = 0.0;
        robot.setAllMotors(botMotion);

        try {
            robot.resetEncoders();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    public abstract void start();
    public abstract void doWork();
    public abstract boolean isComplete();
    public abstract boolean isLost();
    public abstract BotInstruction transition();
    public abstract void fixLost();

    public String getName() {
        return name;
    }
}
