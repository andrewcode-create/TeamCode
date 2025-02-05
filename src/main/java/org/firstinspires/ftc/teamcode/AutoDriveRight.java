package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Disabled
@Autonomous(name = "RIGHTAuto (Blocks to Java)")
public class AutoDriveRight extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Servo clawAngleR;
    private Servo claw;
    private DcMotor slide;
    private DcMotor slideRotate;
    private DcMotor slide2;
    private DcMotor slideRotate2;

    int theta;
    double Controller1LeftStickX;
    double Controller1RightStickX;
    long LastFrameTime;
    double Controller1LeftStickY;
    double Controller1RightStickY;
    int RequestPosNumber;
    boolean DPadDown;
    int CurrentClawPos;
    double SlideRemDist;
    boolean ButtonB;
    boolean ButtonA;

    /**
     * Describe this function...
     */
    private void InitDriveTrain(boolean VelocityMode) {
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
        } else {
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
    private boolean isBetween2(double left, double time2,
                               // TODO: Enter the type for argument named compare
                               double compare) {
        return compare < left + time2 && compare > left;
    }

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue Comment
     * Blocks show where to place Initialization code (runs once, after touching the DS INIT
     * button, and before touching the DS Start arrow), Run code (runs once, after touching
     * Start), and Loop code (runs repeatedly while the OpMode is active, namely not Stopped).
     */
    @Override
    public void runOpMode() {
        boolean LockSlidesAt0;
        boolean LastLockSlidesAt0;
        boolean LastPressed;

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        clawAngleR = hardwareMap.get(Servo.class, "clawAngleR");
        claw = hardwareMap.get(Servo.class, "claw");
        slide = hardwareMap.get(DcMotor.class, "slide");
        slideRotate = hardwareMap.get(DcMotor.class, "slideRotate");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        slideRotate2 = hardwareMap.get(DcMotor.class, "slideRotate2");

        // Put initialization blocks here.
        InitDriveTrain(true);
        InitSlides(true);
        InitSlideRotate(true);
        InitClaw();
        waitForStart();
        LockSlidesAt0 = false;
        LastLockSlidesAt0 = false;
        RequestPosNumber = -1;
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        LastFrameTime = System.currentTimeMillis();
        LastPressed = false;
        resetRuntime();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                SetInputsAutoDriveRight();
                clawAngleR.setPosition(gamepad1.dpad_right ? 0.57 : 0.2);
                // Do the drivetrain. Left bumper is slow mode, right bumper is reverse the robot.
                // REMEMBER Y STICKS ARE REVERSED!
                DriveTrain(gamepad1.right_bumper ? gamepad1.right_stick_x * (gamepad1.right_bumper ? 1 : 1) : Controller1LeftStickX * (gamepad1.right_bumper ? 1 : 1), gamepad1.right_bumper ? -gamepad1.right_stick_y * (gamepad1.right_bumper ? -1 : 1) : -Controller1LeftStickY * (gamepad1.right_bumper ? -1 : 1), gamepad1.right_bumper ? gamepad1.left_stick_x * (gamepad1.right_bumper ? 1 : 1) : Controller1RightStickX * (gamepad1.right_bumper ? 1 : 1), gamepad1.right_bumper ? -gamepad1.left_stick_y * (gamepad1.right_bumper ? -1 : 1) : -Controller1RightStickY * (gamepad1.right_bumper ? -1 : 1), gamepad1.left_bumper ? 0.5 : 1, true);
                // Do the claw
                if (gamepad1.y) {
                    if (!LastPressed) {
                        LastPressed = true;
                        CurrentClawPos = CurrentClawPos == 0 ? 1 : 0;
                    }
                } else {
                    LastPressed = false;
                }
                claw.setPosition(CurrentClawPos);
                // Parse preset position
                if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0 || gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0 || DPadDown || gamepad1.dpad_down || gamepad2.dpad_up || gamepad2.dpad_down) {
                    RequestPosNumber = -1;
                } else if (ButtonA) {
                    RequestPosNumber = 1;
                } else if (ButtonB) {
                    RequestPosNumber = 2;
                } else if (gamepad1.x || gamepad2.x) {
                    RequestPosNumber = 3;
                }
                if (slide.isBusy() || slideRotate.isBusy()) {
                    gamepad1.setLedColor(0.75, 0, 0.87, Gamepad.LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(0.75, 0, 0.87, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                }
                // Do slide rotation
                SlideRotate(RequestPosNumber == -1 ? (gamepad1.left_trigger - gamepad1.right_trigger) + (gamepad2.left_trigger - gamepad2.right_trigger) : 0.7, JavaUtil.createListWith(0, 0, 2600), RequestPosNumber, true, 50, -100, 2700);
                // Do slide extention, including limits for 42 in
                SlideRemDist = GetRemainingSlideDistance(1, 2600, slideRotate.getCurrentPosition(), 2100, 25.5, slide.getCurrentPosition(), 11, 0, 10.5);
                Sync_Extenders();
                if (gamepad1.guide) {
                    if (LastLockSlidesAt0 == false) {
                        LockSlidesAt0 = !LockSlidesAt0;
                        LastLockSlidesAt0 = true;
                    }
                } else {
                    LastLockSlidesAt0 = false;
                }
                if (LockSlidesAt0) {
                    slide.setTargetPosition(0);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(0.8);
                    slide2.setTargetPosition(0);
                    slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide2.setPower(0.8);
                } else {
                    Slides(RequestPosNumber == -1 ? (BoolToInt(gamepad1.dpad_up) - BoolToInt(DPadDown)) + (BoolToInt(gamepad2.dpad_up) - BoolToInt(gamepad2.dpad_down)) : 0.9, JavaUtil.createListWith(0, 1400, 1800), RequestPosNumber, 100, 100, SlideRemDist, true);
                    // Telemetry
                    TelemetryAll();
                    telemetry.update();
                    // This idle call allows other threads to run, like the one that writes to hardware.
                    idle();
                }
            }
        }
    }

    /**
     * Describe this function...
     */
    private boolean isBetween(double left, double right, double compare) {
        return compare < right && compare > left;
    }

    /**
     * Describe this function...
     */
    private void SetInputs(double Runtime2) {
        // ButtonB = raise slide preset
        // ButtonA = wall slide preset
        // DPadDown = lower linear slide when held
        // CurrentClawPos: 0 is open, 1 is close
        if (isBetween(0, 0.6, Runtime2)) {
            // forward
            Controller1LeftStickX = 0;
            Controller1LeftStickY = -0.8;
            Controller1RightStickX = 0;
            Controller1RightStickY = -0.8;
            CurrentClawPos = 1;
        } else if (isBetween(1, 1.5, Runtime2)) {
            // strafe left
            Controller1LeftStickX = -0.8;
            Controller1LeftStickY = 0;
            Controller1RightStickX = -0.8;
            Controller1RightStickY = 0;
            ButtonB = true;
        } else if (isBetween(2, 3.5, Runtime2)) {
            if (Runtime2 < 3.2) {
                // forward
                Controller1LeftStickX = 0;
                Controller1LeftStickY = -0.4;
                Controller1RightStickX = 0;
                Controller1RightStickY = -0.4;
            } else {
                // backward
                Controller1LeftStickX = 0;
                Controller1LeftStickY = 0.4;
                Controller1RightStickX = 0;
                Controller1RightStickY = 0.4;
            }
        } else if (isBetween(3.7, 4.2, Runtime2)) {
            // lower slide
            DPadDown = true;
        } else if (isBetween(4.4, 4.6, Runtime2)) {
            // open claw
            CurrentClawPos = 0;
        } else if (isBetween(4.8, 5.1, Runtime2)) {
            // forward to realign
            Controller1LeftStickX = 0;
            Controller1LeftStickY = -0.8;
            Controller1RightStickX = 0;
            Controller1RightStickY = -0.8;
        } else if (isBetween(5.3, 5.8, Runtime2)) {
            // backward
            Controller1LeftStickX = 0;
            Controller1LeftStickY = 0.8;
            Controller1RightStickX = 0;
            Controller1RightStickY = 0.8;
        } else if (isBetween(5.9, 7.2, Runtime2)) {
            // turn right
            Controller1LeftStickX = 0;
            Controller1LeftStickY = -0.52;
            Controller1RightStickX = 0;
            Controller1RightStickY = 0.52;
            ButtonA = true;
        } else if (isBetween(7.4, 10, Runtime2)) {
            if (Runtime2 < 9) {
                // forward
                Controller1LeftStickX = 0;
                Controller1LeftStickY = -0.8;
                Controller1RightStickX = 0;
                Controller1RightStickY = -0.8;
            } else {
                // forward slow
                Controller1LeftStickX = 0;
                Controller1LeftStickY = -0.5;
                Controller1RightStickX = 0;
                Controller1RightStickY = -0.5;
            }
        } else if (isBetween(10.2, 10.6, Runtime2)) {
            // Backward
            Controller1LeftStickX = 0;
            Controller1LeftStickY = 0.8;
            Controller1RightStickX = 0;
            Controller1RightStickY = 0.8;
        } else if (isBetween(10.8, 11.8, Runtime2)) {
            // turn right
            Controller1LeftStickX = 0;
            Controller1LeftStickY = -0.8;
            Controller1RightStickX = 0;
            Controller1RightStickY = 0.8;
        } else if (isBetween(12, 13, Runtime2)) {
            // strafe left
            Controller1LeftStickX = -0.8;
            Controller1LeftStickY = 0;
            Controller1RightStickX = -0.8;
            Controller1RightStickY = 0;
        } else if (isBetween(13.2, 14.5, Runtime2)) {
            // forward+left
            Controller1LeftStickX = -0.6;
            Controller1LeftStickY = -0.8;
            Controller1RightStickX = -0.6;
            Controller1RightStickY = -0.8;
        } else if (false) {
        } else if (false) {
        } else if (false) {
        } else if (false) {
        } else if (false) {
        } else {
            // Stop
            Controller1LeftStickX = 0;
            Controller1LeftStickY = 0;
            Controller1RightStickX = 0;
            Controller1RightStickY = 0;
            ButtonB = false;
            ButtonA = false;
            DPadDown = false;
        }
    }

    /**
     * Describe this function...
     */
    private void SetInputsAutoDriveLeft() {
        Controller1LeftStickX = -1;
        Controller1RightStickX = -1;
    }

    /**
     * Describe this function...
     */
    private void SetInputsAutoDriveRight() {
        Controller1LeftStickX = 0.4;
        Controller1RightStickX = 0.4;
        Controller1LeftStickY = 0;
        Controller1RightStickY = 0;
    }

    /**
     * Describe this function...
     */
    private void Sync_Extenders() {
        if (gamepad2.left_stick_button && gamepad2.right_stick_button || gamepad1.left_stick_button && gamepad1.right_stick_button) {
            slide.setTargetPosition(slide2.getCurrentPosition());
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.2);
            sleep(1000);
        }
    }

    /**
     * Describe this function...
     */
    private double BoolToInt(boolean Bool) {
        return Bool ? 1 : 0;
    }

    /**
     * Prints useful information to the driver hub. Make sure to call Telemetry.update afterwards.
     */
    private void TelemetryAll() {
        double TimeElapsed;

        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        TimeElapsed = System.currentTimeMillis() - LastFrameTime;
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        LastFrameTime = System.currentTimeMillis();
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("MS update Time", TimeElapsed);
        telemetry.addData("Preset Number", RequestPosNumber);
        telemetry.addData("Remaining Slide Distance", SlideRemDist);
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.addData("Slide Position", slide.getCurrentPosition());
        telemetry.addData("Slide2 Position", slide2.getCurrentPosition());
        telemetry.addData("SlideRotate Position", slideRotate.getCurrentPosition());
        telemetry.addData("SlideRotate2 Position", slideRotate2.getCurrentPosition());
        telemetry.addData("SlideRotate Angle", theta);
    }

    /**
     * Describe this function...
     */
    private void InitClaw() {
        claw.setDirection(Servo.Direction.REVERSE);
        claw.scaleRange(0.71, 0.85);
    }

    /**
     * Describe this function...
     */
    private void InitSlides(boolean VelocityMode) {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);
        if (VelocityMode) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Describe this function...
     */
    private void InitSlideRotate(boolean VelocityMode) {
        slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotate2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotate2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotate.setDirection(DcMotor.Direction.REVERSE);
        slideRotate2.setDirection(DcMotor.Direction.REVERSE);
        if (VelocityMode) {
            slideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Describe this function...
     */
    private void SlideRotate(double RequestedSpeed, List PositionEncoderList, double PositionNum, boolean VelocityMode, int BufferSize, int leftLim, int rightLim) {
        // PositionEncoderList is a list of encoder positions. Position num is the index in the list that the robot is going to. -1 means no position number. Ignores RequestedSpeed if position number is set.
        // Disable if encoders don't match
        if (Math.abs(slideRotate.getCurrentPosition() - slideRotate2.getCurrentPosition()) > 200) {
            telemetry.addData("ERROR", "SlideRotate encoders not synced");
            telemetry.addData("ERROR1 Rotate", slideRotate.getCurrentPosition());
            telemetry.addData("ERROR2 Rotate", slideRotate2.getCurrentPosition());
            telemetry.addData("ERROR RequestedSpeed", RequestedSpeed);
            slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRotate.setPower(0);
            slideRotate2.setPower(0);
            if (true) {
                return;
            }
        }
        if (PositionNum == -1) {
            // Position number not set, allow manual control.
            if (VelocityMode) {
                slideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (slideRotate.getCurrentPosition() > rightLim - BufferSize) {
                // Allow left but not right
                if (false) {
                    ((DcMotorEx) slideRotate).setVelocity(Math.min(RequestedSpeed, 0));
                    ((DcMotorEx) slideRotate2).setVelocity(Math.min(RequestedSpeed, 0));
                } else {
                    slideRotate.setPower(Math.min(RequestedSpeed, 0));
                    slideRotate2.setPower(Math.min(RequestedSpeed, 0));
                }
            } else if (slideRotate.getCurrentPosition() < leftLim + BufferSize) {
                // Allow right but not left
                if (false) {
                    ((DcMotorEx) slideRotate).setVelocity(Math.max(RequestedSpeed, 0));
                    ((DcMotorEx) slideRotate2).setVelocity(Math.max(RequestedSpeed, 0));
                } else {
                    slideRotate.setPower(Math.max(RequestedSpeed, 0));
                    slideRotate2.setPower(Math.max(RequestedSpeed, 0));
                }
            } else {
                if (false) {
                    ((DcMotorEx) slideRotate).setVelocity(RequestedSpeed);
                    ((DcMotorEx) slideRotate2).setVelocity(RequestedSpeed);
                } else {
                    slideRotate.setPower(RequestedSpeed);
                    slideRotate2.setPower(RequestedSpeed);
                }
            }
        } else {
            // Go to position number if not already going
            telemetry.addData("TMP", "!!!!!!");
            if ((int)JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false) != slideRotate.getCurrentPosition()) {
                slideRotate.setTargetPosition(((Integer) JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false)).intValue());
                slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideRotate.setPower(RequestedSpeed);
                telemetry.addData("TMP", "Going1");
            } else {
                telemetry.addData("TMP", "bad 1");
            }
            if ((int)JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false) != slideRotate2.getCurrentPosition()) {
                slideRotate2.setTargetPosition(((Integer) JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false)).intValue());
                slideRotate2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideRotate2.setPower(RequestedSpeed);
                telemetry.addData("TMP", "Going2");
            } else {
                telemetry.addData("TMP", "bad 2");
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Slides(double RequestedSpeed, List PositionEncoderList, double PositionNum, int BufferSize, int MaxBufferSize, double remSlideDist, boolean VelocityMode) {
        // PositionEncoderList is a list of encoder positions. Position num is the index in the list that the robot is going to. -1 means no position number. Ignores RequestedSpeed if position number is set.
        // If Encoders don't match, error
        if (Math.abs(slide.getCurrentPosition() - slide2.getCurrentPosition()) > 100) {
            telemetry.addData("ERROR", "Slide encoders not synced");
            telemetry.addData("ERROR1 Slide", slide.getCurrentPosition());
            telemetry.addData("ERROR2 Slide", slide2.getCurrentPosition());
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(0);
            slide2.setPower(0);
            if (true) {
                return;
            }
        }
        if (PositionNum == -1) {
            // Position number not set, allow manual control.
            if (VelocityMode) {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (remSlideDist < MaxBufferSize * 0.5) {
                // Force Retract Slide
                if (false) {
                    ((DcMotorEx) slide).setVelocity(-1);
                    ((DcMotorEx) slide2).setVelocity(-1);
                } else {
                    slide.setPower(-1);
                    slide2.setPower(-1);
                }
            } else if (remSlideDist < MaxBufferSize) {
                // Allow Retraction but not expansion
                if (false) {
                    ((DcMotorEx) slide).setVelocity(Math.min(RequestedSpeed, 0));
                    ((DcMotorEx) slide2).setVelocity(Math.min(RequestedSpeed, 0));
                } else {
                    slide.setPower(Math.min(RequestedSpeed, 0));
                    slide2.setPower(Math.min(RequestedSpeed, 0));
                }
            } else if (slide.getCurrentPosition() < BufferSize) {
                // Allow Expansion but not retraction
                if (false) {
                    ((DcMotorEx) slide).setVelocity(Math.max(RequestedSpeed, 0));
                    ((DcMotorEx) slide2).setVelocity(Math.max(RequestedSpeed, 0));
                } else {
                    slide.setPower(Math.max(RequestedSpeed, 0));
                    slide2.setPower(Math.max(RequestedSpeed, 0));
                }
            } else {
                // Allow both retraction and expansion
                if (false) {
                    ((DcMotorEx) slide).setVelocity(RequestedSpeed);
                    ((DcMotorEx) slide2).setVelocity(RequestedSpeed);
                } else {
                    slide.setPower(RequestedSpeed);
                    slide2.setPower(RequestedSpeed);
                }
            }
        } else {
            if (remSlideDist < 0) {
                // If past allowed position, cancel move to positon and retract
                if (VelocityMode) {
                    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if (false) {
                    ((DcMotorEx) slide).setVelocity(-1);
                    ((DcMotorEx) slide2).setVelocity(-1);
                } else {
                    slide.setPower(-1);
                    slide2.setPower(-1);
                }
            } else if (remSlideDist < BufferSize && (int)JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false) > slide.getCurrentPosition()) {
                // If within buffer and moving up, cancel movement.
                if (VelocityMode) {
                    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if (false) {
                    ((DcMotorEx) slide).setVelocity(0);
                    ((DcMotorEx) slide2).setVelocity(0);
                } else {
                    slide.setPower(0);
                    slide2.setPower(0);
                }
            } else {
                // Go to position number if not already going
                if ((int)JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false) != slide.getCurrentPosition()) {
                    slide.setTargetPosition(((Integer) JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false)).intValue());
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(RequestedSpeed);
                }
                if ((int)JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false) != slide2.getCurrentPosition()) {
                    slide2.setTargetPosition(((Integer) JavaUtil.inListGet(PositionEncoderList, JavaUtil.AtMode.FROM_START, (int) (PositionNum - 1), false)).intValue());
                    slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide2.setPower(RequestedSpeed);
                }
            }
        }
    }

    /**
     * Describe this function...
     */
    private void DriveTrain(double x1, double y1, double x2, double y2, double speed, boolean VelocityMode) {
        double br;
        double fr;
        double bl;
        double fl;
        double tmp;

        // Controls the Drive Train
        br = y2 + x2;
        fr = y2 - x2;
        bl = y1 - x1;
        fl = y1 + x1;
        tmp = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(br), Math.abs(fr), Math.abs(bl), Math.abs(fl), 1));
        br = (br / tmp) * speed;
        fr = (fr / tmp) * speed;
        bl = (bl / tmp) * speed;
        fl = (fl / tmp) * speed;
        if (false) {
            ((DcMotorEx) backRight).setVelocity(br);
            ((DcMotorEx) frontRight).setVelocity(fr);
            ((DcMotorEx) backLeft).setVelocity(bl);
            ((DcMotorEx) frontLeft).setVelocity(fl);
        } else {
            backRight.setPower(br);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            frontLeft.setPower(fl);
        }
    }

    /**
     * Describe this function...
     */
    private double GetRemainingSlideDistance(int SlideRotate_Vertical_Encoder, int SlideRotate_Horizontal_Encoder, int Current_SlideRotate_Encoder, int Max_Slide_Encoder, double Max_Slide_Length, int Current_Slide_Encoder, int Slide_Position_in_Robot__inches_from_back_, int Slide_Allowed_Offset__in_slide_can_go_past_back_of_robot_, double Arm_length__in_) {
        double ret;

        // This function gets the remaining encoder value allowed for the slide to extend for the given slideRotate encoder position.
        // This includes limits on max slide encoding allowed.
        // This will be negative if the slide is past the allowed distance.
        theta = 90 * (Current_SlideRotate_Encoder / (SlideRotate_Vertical_Encoder - SlideRotate_Horizontal_Encoder) + SlideRotate_Horizontal_Encoder / (SlideRotate_Horizontal_Encoder - SlideRotate_Vertical_Encoder));
        if (theta < 90) {
            // arm in front of pivot
            ret = ((37 - (Slide_Position_in_Robot__inches_from_back_ + Slide_Allowed_Offset__in_slide_can_go_past_back_of_robot_)) - (Arm_length__in_ + (Max_Slide_Length / Max_Slide_Encoder) * Current_Slide_Encoder) * Math.cos(theta / 180 * Math.PI)) / Math.cos(theta / 180 * Math.PI);
        } else if (theta > 90) {
            // arm in behind pivot, ie. angle >90
            ret = (Slide_Position_in_Robot__inches_from_back_ + Slide_Allowed_Offset__in_slide_can_go_past_back_of_robot_ + (Arm_length__in_ + (Max_Slide_Length / Max_Slide_Encoder) * Current_Slide_Encoder) * Math.cos(theta / 180 * Math.PI)) / -Math.cos(theta / 180 * Math.PI);
        } else {
            // arm is vertical, so set ret to a very large number. Fixes divide by 0 error giving negative number at cos(90) = 0.
            // this number is a little under 2^31.
            ret = Math.pow(10, 9);
        }
        return Math.min(ret * (Max_Slide_Encoder / Max_Slide_Length), Max_Slide_Encoder - Current_Slide_Encoder);
    }
}
