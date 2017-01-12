package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Mike on 10/25/2016.
 */

public class BotMotion {
    public long currentTime = 0;
    public double elapsedTime = 0;
    public double X_Position_Inches;
    public boolean maxDrivePowerAchieved;
    public double maxdrivePower;
    public Velocity velocity;
    public double ms;
    public double timeElapsed;
    public double targetHeading;
    public double normalizedHeading;
    public double headingError;
    public double newRightMotorPower;
    public double newLeftMotorPower;
    public double newTurningSpeed;
    public double turningSpeed = 0.088;
    public double turningPowercopy = 0.0;

    /**
     * If the is true we are going to change the ball motor from one state to another.
     */
    public boolean shouldBallMotor = true;
    /**
     * If true we are requesting to ramp up, if false we are requesting to ramp down.
     */
    public boolean isBallMotorOn = false;

    public int leftBallMotorTicks = 0;
    public int rightBallMotorTicks = 0;


    public boolean isElevatorOn = false;
    public double newBallMotorSpeed = 0.0;
    public double newElevatorSpeed = 0.0;
    public double initialPitchValue = 0.0;
    public double newSweeperSpeed = 0.0;
    public Orientation angles;
    public int isSweepRev = 0;



}
