package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private boolean VMode;

    public void init(boolean VelocityMode, DcMotor back_Left, DcMotor back_Right, DcMotor front_Left, DcMotor front_Right) {
        backLeft = back_Left;
        backRight = back_Right;
        frontLeft = front_Left;
        frontRight = front_Right;
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (VelocityMode) {
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VMode = true;
        } else {
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            VMode = false;
        }
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // returns a clamped value for turn speed in [-1,1], given an angle difference
    private double transformRotation(double diff) {
        final double modifier = 4;
        // see https://www.desmos.com/calculator/ujxklm6xd1
        return (2.0/(1+Math.pow(Math.E, -diff*modifier))-1);
    }

    // takes in mm
    private double transformDrive(double diff) {
        final double modifier = 0.1;
        // see https://www.desmos.com/calculator/ujxklm6xd1
        return (2.0/(1+Math.pow(Math.E, -diff*modifier))-1);
    }

    public void DriveToPoint(double toX, double toY, double toAngle, double botHeading, double botX, double botY, double speed) {
        double driveX = transformDrive(Math.abs(toX - botX));
        double driveY = transformDrive(Math.abs(toY - botY));

        DriveFieldCentric(driveX, driveY, toAngle, botHeading, speed);
    }

    // go to angle of turnX and turnY from current angle
    public void DriveFieldCentric(double driveX, double driveY, double toAngle, double botHeading, double speed) {
        // rotate the drive constants
        double X = driveX * Math.cos(-botHeading) - driveY * Math.sin(-botHeading);
        double Y = driveX * Math.sin(-botHeading) + driveY * Math.cos(-botHeading);

        X = X * 1.07;  // Counteract imperfect strafing


        int rDirection = toAngle > botHeading ? -1 : 1;

        // get desired rotation speed and clamp it to [-1,1]
        double r = rDirection * Math.abs(transformRotation(toAngle - botHeading));

        double fl = Y + X + r;
        double bl = Y - X + r;
        double fr = Y - X - r;
        double br = Y + X - r;

        // scale everything to [-1,1]
        double denominator = (1/speed)*(Math.max(Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))), 1));

        fl /= denominator;
        bl /= denominator;
        fr /= denominator;
        br /= denominator;

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
}
