package org.firstinspires.ftc.teamcode.instructions;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.WiredHardware;

public class Turn extends BotInstruction {
    private boolean turnLeft;
    private double targetHeading;

    public Turn(String stateName, boolean turnLeft, Long targetHeading, BotInstruction nextState, WiredHardware robot, BotMotion botMotion) {
        super(stateName, nextState, robot, botMotion);
        this.turnLeft = turnLeft;

        this.targetHeading = targetHeading;
    }

    @Override
    public void start() {
        // commented out since we need to always have this be a reference value from the start of the program
        //botMotion.YawAngleOffset = Utils.convertheading(robot.getAngles()[0]);
        this.stopRobot();
    }

    @Override
    public void doWork() {
        if (turnLeft) {
            Utils.turnLeft(botMotion, robot);
        } else {
            Utils.turnRight(botMotion, robot);
        }
    }

    @Override
    public boolean isComplete() {
        if (turnLeft) {
            if (this.targetHeading > 180 && botMotion.normalizedHeading < 180) {
                return false;
            } else if (this.targetHeading > botMotion.normalizedHeading) {
                return true;
            } else {
                return false;
            }
         } else {
            if (this.targetHeading < 180 && botMotion.normalizedHeading > 180) {
                return false;
            } else if (this.targetHeading < botMotion.normalizedHeading) {
                return true;
            } else {
                return false;
            }
        }

    }

    @Override
    public boolean isLost() {
        return botMotion.normalizedHeading > this.targetHeading + 5;
    }

    @Override
    public BotInstruction transition() {
        botMotion.targetHeading = this.targetHeading;
        //botMotion.maxDrivePowerAchieved = false;
        botMotion.timeElapsedMotion = 0;

        try {
            robot.resetEncoders();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return nextState;
    }

    @Override
    public void fixLost() {

    }
}
