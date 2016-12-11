package org.firstinspires.ftc.teamcode.instructions;

/**
 * Created by Mike on 11/12/2016.
 */

public class Stop extends BotInstruction {

    public Stop() {
        super("Stop", null, null, null);
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
        return null;
    }

    @Override
    public void fixLost() {

    }
}
