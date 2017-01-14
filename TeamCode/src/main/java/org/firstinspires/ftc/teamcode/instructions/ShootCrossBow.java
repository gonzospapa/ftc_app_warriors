package org.firstinspires.ftc.teamcode.instructions;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.WiredHardware;

/**
 * Created by Mike on 1/14/2017.
 */

public class ShootCrossBow extends BotInstruction {

   private long timeatstart=0;
    public ShootCrossBow(String stateName, BotInstruction nextState, WiredHardware robot, BotMotion botMotion) {
        super(stateName, nextState, robot, botMotion);
        this.robot = robot;
        this.botMotion = botMotion;

    }

    @Override
    public void start() {
        this.stopRobot();
        botMotion.isBallMotorOn = true;
        botMotion.shouldBallMotor = true;
        this.timeatstart = botMotion.currentTime;
    }

    @Override
    public void doWork() {
        robot.setBallMotor(botMotion);
        botMotion.isBallMotorOn = false;
        botMotion.shouldBallMotor = false;

    }

    @Override
    public boolean isComplete() {
        boolean retVal = false;
        if ((botMotion.currentTime - this.timeatstart) >= 5000) {
            this.stopRobot();
            botMotion.isBallMotorOn = false;
            botMotion.shouldBallMotor = false;
            retVal = true;
        }
        return retVal;
    }

    @Override
    public boolean isLost() {
        return false;
    }

    @Override
    public BotInstruction transition() {
        return null;
    }

    @Override
    public void fixLost() {

    }
}
