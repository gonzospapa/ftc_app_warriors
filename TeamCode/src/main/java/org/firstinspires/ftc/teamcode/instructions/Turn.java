package org.firstinspires.ftc.teamcode.instructions;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.WiredHardware;

public class Turn extends BotInstruction {
    private boolean turnLeft;
    private Long targetHeading;

    public Turn(String stateName, boolean turnLeft, Long targetHeading, BotInstruction nextState, WiredHardware robot, BotMotion botMotion) {
        super(stateName, nextState, robot, botMotion);
        this.turnLeft = turnLeft;
        this.targetHeading = targetHeading;
    }

    @Override
    public void start() {
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
        return botMotion.normalizedHeading >= this.targetHeading;
    }

    @Override
    public boolean isLost() {
        return botMotion.normalizedHeading > this.targetHeading + 5;
    }

    @Override
    public BotInstruction transition() {
        return nextState;
    }

    @Override
    public void fixLost() {

    }
}
