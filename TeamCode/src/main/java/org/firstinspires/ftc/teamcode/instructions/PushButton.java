package org.firstinspires.ftc.teamcode.instructions;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.WiredHardware;

/**
 * Created by Mike on 11/12/2016.
 */

public class PushButton extends BotInstruction {

    private BotInstruction nextState;

    public PushButton(String stateName, BotInstruction nextState, WiredHardware robot, BotMotion botMotion) {
        super(stateName, nextState, robot, botMotion);
        this.nextState = nextState;
    }

    @Override
    public void start() {

    }

    @Override
    public void doWork() {

    }

    @Override
    public boolean isComplete() {
        return false;
    }

    @Override
    public boolean isLost() {
        return false;
    }

    @Override
    public BotInstruction transition() {
        return nextState;
    }

    @Override
    public void fixLost() {

    }
}
