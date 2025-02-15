package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DriveTrain {
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;

    private boolean VMode;

    public void init(boolean VelocityMode, DcMotorEx back_Left, DcMotorEx back_Right, DcMotorEx front_Left, DcMotorEx front_Right) {
        backLeft = back_Left;
        backRight = back_Right;
        frontLeft = front_Left;
        frontRight = front_Right;
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        if (VelocityMode) {
            backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            VMode = true;
        } else {
            backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            VMode = false;
        }
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // returns a clamped value for turn speed in [-1,1], given an angle difference
    private double transformRotation(double diff) {
        final double modifier = 2.5;
        // see https://www.desmos.com/calculator/ujxklm6xd1
        return (2.0/(1+Math.pow(Math.E, -diff*modifier))-1);
    }

    // takes in mm
    // TODO sqrt() would be better
    private double transformDrive(double diff) {
        final double modifier = 0.0035;
        // see https://www.desmos.com/calculator/ujxklm6xd1
        return clamp(modifier/2*diff, -1, 1);
        //return (2.0/(1+Math.pow(Math.E, -diff*modifier))-1);
    }

    private double transformDriveSqrt(double diff) {
        final double modifier = 0.0035;
        // see https://www.desmos.com/calculator/ujxklm6xd1
        return clamp(modifier*10*Math.sqrt(Math.abs(diff))*(diff < 0 ? -1 : 1), -1, 1);
        //return (2.0/(1+Math.pow(Math.E, -diff*modifier))-1);
    }

    private double transformDriveFast(double diff) {
        return clamp(transformDrive(diff)*2, -1, 1);
    }

    public static double getDistanceToPoint(Pose2D to, Pose2D bot) {
        return Math.sqrt(
                (to.getX(DistanceUnit.MM)-bot.getX(DistanceUnit.MM))*(to.getX(DistanceUnit.MM)-bot.getX(DistanceUnit.MM))
                + (to.getY(DistanceUnit.MM)-bot.getY(DistanceUnit.MM))*(to.getY(DistanceUnit.MM)-bot.getY(DistanceUnit.MM))
        );
    }
    public static double getDistanceToPointX(Pose2D to, Pose2D bot) {
        return Math.abs(to.getX(DistanceUnit.MM)-bot.getX(DistanceUnit.MM));
    }
    public static double getDistanceToPointY(Pose2D to, Pose2D bot) {
        return Math.abs(to.getY(DistanceUnit.MM)-bot.getY(DistanceUnit.MM));
    }

    public void DriveToPoint(Pose2D to, Pose2D bot, double speed) {
        double stickY = transformDriveSqrt(to.getX(DistanceUnit.MM) - bot.getX(DistanceUnit.MM));
        double stickX = -transformDriveSqrt(to.getY(DistanceUnit.MM) - bot.getY(DistanceUnit.MM));

        DriveFieldCentric(stickX, stickY, to.getHeading(AngleUnit.RADIANS), bot.getHeading(AngleUnit.RADIANS), speed, null);
    }

    public void DriveToPointOld(Pose2D to, Pose2D bot, double speed) {
        double stickY = transformDrive(to.getX(DistanceUnit.MM) - bot.getX(DistanceUnit.MM));
        double stickX = -transformDrive(to.getY(DistanceUnit.MM) - bot.getY(DistanceUnit.MM));

        DriveFieldCentric(stickX, stickY, to.getHeading(AngleUnit.RADIANS), bot.getHeading(AngleUnit.RADIANS), speed, null);
    }
    public void DriveToPointToWall(Pose2D to, Pose2D bot, double speed) {
        double stickY = transformDrive(1.2*(to.getX(DistanceUnit.MM) - bot.getX(DistanceUnit.MM)));
        double stickX = -transformDrive(1.2*(to.getY(DistanceUnit.MM) - bot.getY(DistanceUnit.MM)));

        DriveFieldCentric(stickX, stickY, to.getHeading(AngleUnit.RADIANS), bot.getHeading(AngleUnit.RADIANS), speed, null);
    }
    public void DriveToPointGoThrough(Pose2D to, Pose2D bot, double speed) {
        double stickY = transformDriveSqrt(to.getX(DistanceUnit.MM) - bot.getX(DistanceUnit.MM));
        double stickX = -transformDriveSqrt(to.getY(DistanceUnit.MM) - bot.getY(DistanceUnit.MM));

        DriveFieldCentric(clamp(stickX, -1, 1), clamp(stickY, -1, 1), to.getHeading(AngleUnit.RADIANS), bot.getHeading(AngleUnit.RADIANS), speed, null);
    }
    public void DriveToPointGoThroughOld(Pose2D to, Pose2D bot, double speed) {
        double stickY = transformDriveFast(to.getX(DistanceUnit.MM) - bot.getX(DistanceUnit.MM));
        double stickX = -transformDriveFast(to.getY(DistanceUnit.MM) - bot.getY(DistanceUnit.MM));

        DriveFieldCentric(clamp(stickX, -1, 1), clamp(stickY, -1, 1), to.getHeading(AngleUnit.RADIANS), bot.getHeading(AngleUnit.RADIANS), speed, null);
    }

    /*
    public void DriveToPoint(double toX, double toY, double toAngle, double botHeading, double botX, double botY, double speed) {
        double driveX = transformDrive(toX - botX);
        double driveY = transformDrive(toY - botY);

        DriveFieldCentric(driveX, driveY, toAngle, botHeading, speed, null);
    }
    */

    public void DriveNoEncoder(double axial, double lateral, double yaw, double speed) {
        double max;

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        backRight.setPower(speed*rightBackPower);
        frontRight.setPower(speed*rightFrontPower);
        backLeft.setPower(speed*leftBackPower);
        frontLeft.setPower(speed*leftFrontPower);
    }

    // go to angle of turnX and turnY from current angle
    public void DriveFieldCentric(double driveX, double driveY, double toAngle, double botHeading, double speed, Telemetry telemetry) {
        // rotate the drive constants
        double X = driveX * Math.cos(-botHeading) - driveY * Math.sin(-botHeading);
        double Y = driveX * Math.sin(-botHeading) + driveY * Math.cos(-botHeading);

        X = X * 1.07;  // Counteract imperfect strafing


        //int rDirection = toAngle > botHeading ? -1 : 1;

        // get desired rotation speed and clamp it to [-1,1]
        // positive means turn left, negative means turn right
        double diff = botHeading - toAngle;
        //telemetry.addData("DIFF", diff);
        while (diff > Math.PI) diff -= 2*Math.PI;
        while (diff <= -Math.PI) diff += 2*Math.PI;

        double r = transformRotation(diff);

        double fl = Y + X + r;
        double bl = Y - X + r;
        double fr = Y - X - r;
        double br = Y + X - r;
        /*
        telemetry.addLine("Before scaling:");
        telemetry.addData("fl", fl);
        telemetry.addData("bl", bl);
        telemetry.addData("fr", fr);
        telemetry.addData("br", br);

         */

        // scale everything to [-1,1]
        double denominator = (1/speed)*(Math.max(Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))), 1));

        fl /= denominator;
        bl /= denominator;
        fr /= denominator;
        br /= denominator;/*
        telemetry.addLine("After scaling:");
        telemetry.addData("fl", fl);
        telemetry.addData("bl", bl);
        telemetry.addData("fr", fr);
        telemetry.addData("br", br);
        */


        backRight.setPower(br);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        frontLeft.setPower(fl);
    }

    public void Drive(double x1, double y1, double x2, double y2, double speed) {
        double br = y2 + x2;
        double fr = y2 - x2;
        double bl = y1 - x1;
        double fl = y1 + x1;

        double norm_factor = speed * Math.max(Math.max(Math.max(br, fr), Math.max(bl, fl)), 1);

        br *= norm_factor;
        fr *= norm_factor;
        bl *= norm_factor;
        fl *= norm_factor;

        backRight.setPower(br);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        frontLeft.setPower(fl);
    }



    private final double momentOfInertia = 1.0; // Adjustable turning inertia
    private final double linearInertia = 0.1; // Adjustable inertia for movement

    public enum TurnMode {
        CLOCKWISE, COUNTERCLOCKWISE, SHORTEST
    }


    public void goToPointSmooth(Pose2D toPoint, Pose2D pos, double maxSpeed, TurnMode turnMode, double endVelocity) {


        double targetX = toPoint.getX(DistanceUnit.MM);
        double targetY = toPoint.getY(DistanceUnit.MM);
        double targetAngle = toPoint.getHeading(AngleUnit.DEGREES);
        double currentX = pos.getX(DistanceUnit.MM);
        double currentY = pos.getY(DistanceUnit.MM);
        double currentAngle = pos.getHeading(AngleUnit.DEGREES);


        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);


        double targetDirection = Math.toDegrees(Math.atan2(deltaY, deltaX));
        //double movementError = normalizeAngle(targetDirection - currentAngle);

        double speed = Math.min(distance * linearInertia + endVelocity, maxSpeed);

        double turnError = normalizeAngle(targetAngle - currentAngle);
        if (turnMode == TurnMode.CLOCKWISE && turnError > 0) turnError -= 360;
        if (turnMode == TurnMode.COUNTERCLOCKWISE && turnError < 0) turnError += 360;
        if (turnMode == TurnMode.SHORTEST) {
            if (turnError > 180) turnError -= 360;
            if (turnError < -180) turnError += 360;
        }

        double turnSpeed = (1 - Math.exp(-Math.abs(turnError) / 90)) * momentOfInertia * Math.signum(turnError);

        double strafeX = Math.cos(Math.toRadians(targetDirection)) * speed;
        double strafeY = Math.sin(Math.toRadians(targetDirection)) * speed;

        double fl = strafeX + strafeY + turnSpeed;
        double bl = strafeX - strafeY + turnSpeed;
        double fr = strafeX - strafeY - turnSpeed;
        double br = strafeX + strafeY - turnSpeed;

        double denominator = (Math.max(Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))), 1));


        fl /= denominator;
        bl /= denominator;
        fr /= denominator;
        br /= denominator;

        // Omni-wheel kinematics
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }



    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
