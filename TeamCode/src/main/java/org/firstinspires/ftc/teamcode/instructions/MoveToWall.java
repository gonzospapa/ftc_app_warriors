package org.firstinspires.ftc.teamcode.instructions;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.WiredHardware;

/**
 * Created by Mike on 1/18/2017.
 */

public class MoveToWall extends BotInstruction {
    private boolean mforward;
    private Long targetDistance;

    public MoveToWall(String stateName, boolean forward, Long distance, BotInstruction nextState, WiredHardware robot, BotMotion botMotion) {
        super(stateName, nextState, robot, botMotion);
        this.mforward = forward;
        this.targetDistance = distance;
        this.robot = robot;
        this.botMotion = botMotion;
    }

    @Override
    public void start() {
        this.stopRobot();
    }

    @Override
    public void doWork() {
        Utils.goStraightKeepStraight(botMotion, robot, targetDistance);
    }

    @Override
    public boolean isComplete() { return botMotion.X_Position_Inches >= targetDistance || Utils.AreWeInFrontOfWall(this.robot);
    }

    @Override
    public boolean isLost() {
        return botMotion.X_Position_Inches > targetDistance + 5;
    }

    @Override
    public BotInstruction transition() {
        return this.nextState;
    }

    @Override
    public void fixLost() {
        //Backup
    }
}