package org.firstinspires.ftc.teamcode.instructions;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BotMotion;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.WiredHardware;

/**
 * Created by Mike on 11/12/2016.
 */

public class PushButton extends BotInstruction {

    private BotInstruction nextState;
    private boolean lookingForBlue = false;
    private boolean isComplete = false;

    public PushButton(String stateName, BotInstruction nextState, WiredHardware robot, BotMotion botMotion, boolean areWeLookingForBlue) {
        super(stateName, nextState, robot, botMotion);
        this.nextState = nextState;
        lookingForBlue = areWeLookingForBlue;
    }

    @Override
    public void start() {

    }

    @Override
    public void doWork() {
        if (lookingForBlue) {
            if (Utils.IsBlue(robot)) {
                robot.buttonPusher.setDirection(Servo.Direction.REVERSE);
                robot.buttonPusher.setPosition(.75);
            } else {
                robot.buttonPusher.setDirection(Servo.Direction.FORWARD);
                robot.buttonPusher.setPosition(.75);
            }
        } else {
            if (Utils.IsBlue(robot)) {
                robot.buttonPusher.setDirection(Servo.Direction.FORWARD);
                robot.buttonPusher.setPosition(.75);
            } else {
                robot.buttonPusher.setDirection(Servo.Direction.REVERSE);
                robot.buttonPusher.setPosition(.75);
            }
        }
        isComplete = true;
    }

    @Override
    public boolean isComplete() {
        return isComplete;
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
