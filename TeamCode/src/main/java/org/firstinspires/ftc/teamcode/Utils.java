package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

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

                AdjusteddrivePower = 0.065 + (Math.pow(botMotion.X_Position_Inches + 2, 3)) / 5000.0;
            } else// slow down power as we approach target position
            {
                AdjusteddrivePower = 0.035 + (Math.pow(Distancedifference + 1, 2)) / 5000.0;
            }

            if (AdjusteddrivePower > botMotion.maxdrivePower) {
                botMotion.maxDrivePowerAchieved = true;
                AdjusteddrivePower = botMotion.maxdrivePower;
            }


            if ((botMotion.timeElapsed != 0) && (Math.abs(botMotion.velocity.xVeloc) < 0.025)) {
                if ((botMotion.ms - botMotion.timeElapsed) > 2000) {
                    AdjusteddrivePower = 0.3;
                }
            }

            //timeElapsed was reset. Count time from no movement
            if ((botMotion.timeElapsed == 0) && (Math.abs(botMotion.velocity.xVeloc) < 0.02)) {
                botMotion.timeElapsed = botMotion.ms;// copy global counter
            }
            if (Math.abs(botMotion.velocity.xVeloc) >= 0.01)
                botMotion.timeElapsed = botMotion.ms;// reset if movement occurs

            //HeadingShiftValue shifts the number calculations away from zero to avoid division by zero.
            if (botMotion.targetHeading == 180.0) {
                if (botMotion.normalizedHeading >= 0.0)// if heading is around 180 175
                {
                    botMotion.normalizedHeading = 180 + (180.0 - botMotion.normalizedHeading);
                    botMotion.headingError = ((botMotion.targetHeading - botMotion.normalizedHeading) / (botMotion.targetHeading + 200)) * 2.0;
                } else// normalizedHeading is less than zero
                {
                    botMotion.headingError = (((180 + (180.0 + botMotion.normalizedHeading)) - (botMotion.targetHeading)) / (botMotion.targetHeading + 200)) * 2.0;
                }
            } else {
                botMotion.headingError = ((botMotion.normalizedHeading - botMotion.targetHeading) / (botMotion.targetHeading + 200)) * 2.0;
            }

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

            robot.rightMotor.setPower(botMotion.newRightMotorPower);
            robot.leftMotor.setPower(botMotion.newLeftMotorPower);
        }

    }

    public static Boolean didRoboStoppedtMoving(BotMotion botMotion) {
        if ((Math.abs(botMotion.velocity.xVeloc) < 0.02) && (Math.abs(botMotion.velocity.yVeloc) < 0.02)) {
            botMotion.maxDrivePowerAchieved = false;
            botMotion.timeElapsed = 0;
            return true;
        } else return false;
    }

    public static void turnLeft(BotMotion botMotion, WiredHardware robot)// inverted power application to compensate for gearing.
    {
        botMotion.newTurningSpeed = slowTurning(botMotion);

        robot.rightMotor.setPower(botMotion.newTurningSpeed);
        robot.leftMotor.setPower(-botMotion.newTurningSpeed);
    }

    public static void turnRight(BotMotion botMotion, WiredHardware robot) {
        botMotion.newTurningSpeed = slowTurning(botMotion);
        robot.rightMotor.setPower(-botMotion.newTurningSpeed);
        robot.leftMotor.setPower(botMotion.newTurningSpeed);
    }

    public static void stopRobot(WiredHardware robot) {
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
    }

    public static double slowTurning(BotMotion botMotion) {
        double adjustedTurningSpeed;
        double angularDifference;

        angularDifference = Math.abs((botMotion.normalizedHeading + 200) - (botMotion.targetHeading + 200));
        if (botMotion.maxDrivePowerAchieved == false)// startup power
        {
            adjustedTurningSpeed = botMotion.turningSpeed;
        } else {
            if (botMotion.targetHeading == 180.0) {
                if (botMotion.normalizedHeading >= 0.0)//
                {
                    adjustedTurningSpeed = 0.06 + ((angularDifference * 1.3) / 1000.0);
                } else// normalizedHeading is less than zero i e -179
                {
                    adjustedTurningSpeed = 0.06 + (((180 + botMotion.normalizedHeading) * 1.3) / 1000.0);
                }
            }
            adjustedTurningSpeed = 0.06 + ((angularDifference * 1.3) / 1000.0);
        }

        if (adjustedTurningSpeed >= botMotion.turningSpeed) {
            adjustedTurningSpeed = botMotion.turningSpeed;
            botMotion.maxDrivePowerAchieved = true;
        }

        if ((botMotion.timeElapsed != 0) && (Math.abs(botMotion.velocity.yVeloc) < 0.02)) {
            if ((botMotion.ms - botMotion.timeElapsed) > 1000) {
                adjustedTurningSpeed = botMotion.turningSpeed;
            }
        }

        //timeElapsed was reset. Count time from no movement
        if ((botMotion.timeElapsed == 0) && (Math.abs(botMotion.velocity.yVeloc) < 0.02)) {
            botMotion.timeElapsed = botMotion.ms;// copy global counter
        }

        if (Math.abs(botMotion.velocity.yVeloc) > 0.02)
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
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
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
}


