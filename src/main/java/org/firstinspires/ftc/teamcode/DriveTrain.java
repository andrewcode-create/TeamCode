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
