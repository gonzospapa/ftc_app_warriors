package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * Created by Mike on 10/25/2016.
 */

public final class Utils {

    public static void goStraightKeepStraight(BotMotion botMotion, WiredHardware robot, Long targetDistance) {

        double AdjusteddrivePower = 0L;

        final double Distancedifference = targetDistance - botMotion.X_Position_Inches;

        if (Distancedifference > 0) {
            if (botMotion.maxDrivePowerAchieved == false)// startup power
            {

                AdjusteddrivePower = 0.02 + (Math.pow(botMotion.X_Position_Inches + 1, 2)) / 10000.0;
            } else// slow down power as we approach target position
            {
                AdjusteddrivePower = 0.02 + (Math.pow(Distancedifference + 1, 2)) / 10000.0;
            }

            if (AdjusteddrivePower > botMotion.maxdrivePower) {
                botMotion.maxDrivePowerAchieved = true;
                AdjusteddrivePower = botMotion.maxdrivePower;
            }


            if ((botMotion.timeElapsed != 0) && (Math.abs(botMotion.xVelocity) < 0.025)) {
                if ((botMotion.ms - botMotion.timeElapsed) > 2000) {
                    AdjusteddrivePower = 0.2;
                }
            }

            //timeElapsed was reset. Count time from no movement
            if ((botMotion.timeElapsed == 0) && (Math.abs(botMotion.xVelocity) < 0.02)) {
                botMotion.timeElapsed = botMotion.ms;// copy global counter
            }
            if (Math.abs(botMotion.xVelocity) >= 0.015)
                botMotion.timeElapsed = botMotion.ms;// reset if movement occurs

            botMotion.headingError = ((ComputeAngularDifference(botMotion.normalizedHeading,botMotion.targetHeading)) / (botMotion.targetHeading+70.0)); // adjust for heading correction
            //botMotion.headingError = ((botMotion.normalizedHeading - botMotion.targetHeading) / (botMotion.targetHeading + 200)) * 5.0;


            if (botMotion.headingError > 0.0)// we are skewed to the right. apply more power to that side.
            {
                //if ( headingError > 1.0) headingError = 1.0;
                botMotion.newRightMotorPower = AdjusteddrivePower * (1.0 + botMotion.headingError);
                botMotion.newLeftMotorPower = AdjusteddrivePower * (1.0 - botMotion.headingError);
            } else if (botMotion.headingError <= 0.0)// we are skewed to the left. apply more power to that side.
            {
                //if ( headingError < -1.0) headingError = -1.0;
                botMotion.newRightMotorPower = AdjusteddrivePower * (1.0 + botMotion.headingError);
                botMotion.newLeftMotorPower = AdjusteddrivePower * (1.0 - botMotion.headingError);
            }


            botMotion.newLeftMotorPower = botMotion.newLeftMotorPower;
            botMotion.newRightMotorPower =  botMotion.newRightMotorPower;
            robot.setAllMotors(botMotion);

        }

    }

    public static Boolean didRoboStoppedtMoving(BotMotion botMotion, WiredHardware robot) {
        botMotion.xAcceleration= robot.getAcclerations()[0]; //
        Utils.IntegrateAcceleration(botMotion);// updates integration to get velocity
        if ((Math.abs(botMotion.xVelocity) < 0.02) && (Math.abs(botMotion.xVelocity) < 0.02)) {
            botMotion.maxDrivePowerAchieved = false;
            botMotion.timeElapsed = 0;
            return true;
        } else return false;
    }

    public static void turnLeft(BotMotion botMotion, WiredHardware robot)// inverted power application to compensate for gearing.
    {
        botMotion.newTurningSpeed = slowTurning(botMotion);

        //robot.rightMotor.setPower(botMotion.newTurningSpeed);
        //robot.leftMotor.setPower(-botMotion.newTurningSpeed);


        botMotion.newRightMotorPower =  botMotion.newTurningSpeed;
        botMotion.newLeftMotorPower = -botMotion.newTurningSpeed;
        robot.setAllMotors(botMotion);

    }

    public static void turnRight(BotMotion botMotion, WiredHardware robot) {
        botMotion.newTurningSpeed = slowTurning(botMotion);


        botMotion.newLeftMotorPower = botMotion.newTurningSpeed;
        botMotion.newRightMotorPower =  -botMotion.newTurningSpeed;
        robot.setAllMotors(botMotion);

        //robot.rightMotor.setPower(-botMotion.newTurningSpeed);
        //robot.leftMotor.setPower(botMotion.newTurningSpeed);
    }

    public static void stopRobot(WiredHardware robot) {
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
    }

    public static double slowTurning(BotMotion botMotion) {
        double adjustedTurningSpeed;
        double angularDifference;

        angularDifference = Math.abs(ComputeAngularDifference(botMotion.normalizedHeading,botMotion.targetHeading));
        if (botMotion.maxDrivePowerAchieved == false)// startup power
        {
            adjustedTurningSpeed = botMotion.turningSpeed;
        } else {

            //adjustedTurningSpeed = 0.06 + ((angularDifference * 1.1) / 1000.0);
            adjustedTurningSpeed = 0.035 + ((angularDifference) / 1000.0);
        }

        if (adjustedTurningSpeed >= botMotion.turningSpeed) {
            adjustedTurningSpeed = botMotion.turningSpeed;
            botMotion.maxDrivePowerAchieved = true;
        }

        if ((botMotion.timeElapsed != 0) && (Math.abs(botMotion.xVelocity) < 0.02)) {
            if ((botMotion.ms - botMotion.timeElapsed) > 1000) {
                adjustedTurningSpeed = botMotion.turningSpeed;
            }
        }

        //timeElapsed was reset. Count time from no movement
        if ((botMotion.timeElapsed == 0) && (Math.abs(botMotion.xVelocity) < 0.02)) {
            botMotion.timeElapsed = botMotion.ms;// copy global counter
        }

        if (Math.abs(botMotion.xVelocity) > 0.02)
            botMotion.timeElapsed = botMotion.ms;// reset if movement occurs


        botMotion.turningPowercopy = adjustedTurningSpeed;
        return adjustedTurningSpeed;
    }

    public static double normalizePitchReading(double PitchReading, double OffsetValue)
    {
        double AdjustedPitch;
        AdjustedPitch = normalizeDegrees((PitchReading+200)-(OffsetValue+200));

        return AdjustedPitch;
    }

    /** Normalize the angle into the range [-180,180) */
    public static double normalizeDegrees(double degrees)
    {
        while (degrees >= 359.0) degrees -= 360.0;
        while (degrees < -359.0) degrees += 360.0;
        return degrees;
    }

    public static double getTicksPerSecond(int elapsedTicks, double elapsedTimeSec) {
        //int elapsedTicks = currentPos - currentTicks;
        //currentTicks = currentPos;
        double ticksPerSec = 0;
        if (elapsedTicks != 0 && elapsedTimeSec != 0) {
            ticksPerSec = elapsedTicks / elapsedTimeSec;
        }
        return ticksPerSec;
    }

    public static void setTime(BotMotion botMotion) {
        double elapsedTime = 0;
        long now = System.currentTimeMillis();
        elapsedTime = (now - botMotion.currentTime);
        botMotion.currentTime = now;
        botMotion.ms = now;
        botMotion.elapsedTime = elapsedTime;
        botMotion.timeElapsed = elapsedTime;
    }

    public static double convertEncoderDatToInches (double encoderVal) {
        return (encoderVal)/80.0;
    }

    double get_ods_side_value_in_inches (HardwarePushbot robot)
    {
        //0.0237*G6^(-1.164)
        //double ExpProduct;
        //double dist;
        //ExpProduct = Math.pow(v_sensor_ods_SIDE.getLightDetected(), -0.589);
        //dist = (0.2117 *ExpProduct);
        //if ((dist <20.0) && (dist >=0.0)) return dist;
        //else return 20.0;
        return 20.0;
    }

    public static void delay(long ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch (InterruptedException e)
        {
            //Fix this up.
            //this.(e);
        }
    }

    public static void IntegrateAcceleration(BotMotion botMotion) {
        double elapsedTime = 0;
        double duration = (botMotion.timeElapsed) * 1e-6;

        if (Math.abs(botMotion.xAcceleration) > 0.01)
        {
            botMotion.xVelocity = (botMotion.xAcceleration+botMotion.xAccelerationLast)*0.5*duration;

            botMotion.xAccelerationLast = botMotion.xAcceleration;
        }
        else
        {
            botMotion.xVelocity = 0;
            botMotion.xAccelerationLast = 0;
        }

    }

    public static double convertheading(double inputHeading) {

        while (inputHeading < 0.0) inputHeading += 360.0;
        while (inputHeading >= 359.0) inputHeading -= 360.0;

        return inputHeading;
    }
    /** Apply angle offset to angle reading to make angle readings relative to initial oritentation  */
    public static double ApplyAngleOffset(double AngleReading, double Offsetangle)
    {
        double AdjustedAngle;
        AdjustedAngle = Utils.convertheading(-(AngleReading-Offsetangle));

        return AdjustedAngle;
    }

    public static double ComputeAngularDifference(double AngleReadingOne, double AngleReadingTwo)
    {
        double AngleDifference;

        //convert back to standard angle notations (0 to 180, -1 to -179)
        if (AngleReadingOne > 180)
        {
            AngleReadingOne -=360;
        }

        if (AngleReadingTwo > 180)
        {
            AngleReadingTwo -=360;
        }

        //compensate for wrap around
        AngleDifference =  AngleReadingOne - AngleReadingTwo;
        while (AngleDifference < -180) AngleDifference += 360;
        while (AngleDifference > 180) AngleDifference -= 360;

        return AngleDifference;
    }

    public static boolean AreWeInFrontOfWall(WiredHardware robot)
    {
        if (robot.odsSensor.getRawLightDetected() > 1.7)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public static boolean IsWhiteLineThere(WiredHardware robot)
    {
        if (robot.odsSensorForLineDetect.getRawLightDetected() > 0.012) // NEED TO FIND OUT WHAT THIS VALUE IS
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    public static boolean IsBlue(WiredHardware robot) // If its not blue, then it must be red. what else.
    {
        if ((robot.sensorRGB.red()/robot.sensorRGB.blue()) < 2.0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }




}
