package org.firstinspires.ftc.teamcode.instructions;

import org.firstinspires.ftc.teamcode.WiredHardware;

/**
 * Created by Mike on 11/12/2016.
 */

public class Stop extends BotInstruction {

    public Stop(WiredHardware robot) {
        super("Stop", null, robot, null);
    }

    @Override
    public void start() {

    }

    @Override
    public void doWork() {
        robot.stopAllMovement();
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
        return null;
    }

    @Override
    public void fixLost() {

    }
}
