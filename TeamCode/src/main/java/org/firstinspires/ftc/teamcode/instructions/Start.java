package org.firstinspires.ftc.teamcode.instructions;

/**
 * Created by Mike on 11/12/2016.
 */

    public class Start extends BotInstruction {

    public BotInstruction nextState;

    public Start(BotInstruction nextState) {
        super("Start", nextState, null, null);
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
        return true;
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
