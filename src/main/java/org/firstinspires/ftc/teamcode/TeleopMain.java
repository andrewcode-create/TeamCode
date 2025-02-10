package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        // init slides and rotate
        SlidesAndRotate slidesAndRotate = new SlidesAndRotate(true, true);
        slidesAndRotate.initSlide(hardwareMap.get(DcMotor.class, "slideLeft"),
                hardwareMap.get(DcMotor.class, "slideRight"), false);
        slidesAndRotate.initRotate(hardwareMap.get(DcMotor.class, "slideRotateLeft"),
                hardwareMap.get(DcMotor.class, "slideRotateRight"), false);
        SlidesAndRotate.Presets currentPreset = null;
        //String currentPreset2 = "none";

        // init drivetrain
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.init(true,
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"));

        // init claw
        CustomServo claw = new CustomServo(0.7, 0.42);
        CustomServo clawRotate = new CustomServo(0, 1, 0.475, 0.008);
        claw.init(hardwareMap.get(Servo.class, "claw"), CustomServo.Position.none);
        clawRotate.init(hardwareMap.get(Servo.class, "clawRotate"), CustomServo.Position.mid);

        // set manual sensor caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // define gamepad 1 and 2
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // set telemetry to update the driver station 10 times per second
        telemetry.setMsTransmissionInterval(100);

        // odometer start
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(Constants.pinpointRotX, Constants.pinpointRotY);



        //odo.resetPosAndIMU();
        //telemetry.addLine("Waiting for odometer");
        //telemetry.update();
        //sleep(500);
        // set position to starting teleop position and heading
        //odo.setPosition(new Pose2D(DistanceUnit.MM, Constants.startingPosTeleop.getX(DistanceUnit.MM), Constants.startingPosTeleop.getY(DistanceUnit.MM), AngleUnit.RADIANS, Constants.startingPosTeleop.getHeading(AngleUnit.RADIANS)));
        //sleep(20);


        // Ready!
        telemetry.addLine("Ready to play, everything initialized.");
        telemetry.update();

        waitForStart();

        odo.update();
        Pose2D pos = odo.getPosition();
        claw.moveToPos(CustomServo.Position.close);

        double lastR = pos.getHeading(AngleUnit.RADIANS);

        long LastFrameTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            long TimeElapsed = System.currentTimeMillis() - LastFrameTime;
            LastFrameTime = System.currentTimeMillis();

            // do bulk read for performance improvement
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // odometer update
            odo.update();

            pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            // Store the gamepad values from the previous loop iteration in previous gamepad1/2 to be used in this loop iteration.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being used in the same loop iteration.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // do resetHeading if necessary
            /*
            if (currentGamepad1.touchpad && !previousGamepad1.touchpad || currentGamepad2.touchpad && !previousGamepad2.touchpad) {
                telemetry.addLine("RECALIBRATING IMU");
                telemetry.update();
                driveTrain.Drive(0,0,0,0,0);
                sleep(1000);
                odo.recalibrateIMU();
                sleep(500);
            }*/

            // do sync if necessary
            if (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button || currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
                telemetry.addLine("SYNCING FOR 1 SECOND!!!");
                telemetry.update();
                // THIS WILL BLOCK FOR 1 SECOND!!!!
                slidesAndRotate.Sync();
                // restart loop because inputs are outdated.
                continue;
            }

            // change presets if needed
            if (currentGamepad1.left_trigger!=0 || currentGamepad1.right_trigger!=0 || currentGamepad1.dpad_up || currentGamepad1.dpad_down ||
                    currentGamepad2.left_trigger!=0 || currentGamepad2.right_trigger!=0 || currentGamepad2.dpad_up || currentGamepad2.dpad_down){
                currentPreset = null;
            } else if ( currentGamepad2.a && !previousGamepad2.a) {
                currentPreset = SlidesAndRotate.Presets.WallPickup;
            } else if (currentGamepad2.b && !previousGamepad2.b) {
                currentPreset = SlidesAndRotate.Presets.TopSpecimen;
            } else if (currentGamepad2.square && !previousGamepad2.square) {
                currentPreset = SlidesAndRotate.Presets.Middle;
            } else if (currentGamepad1.guide && !previousGamepad1.guide || currentGamepad2.guide && !previousGamepad2.guide) {
                currentPreset = SlidesAndRotate.Presets.Ascent;
            }

            // do slide rotation and extension
            if (currentPreset == null) {
                slidesAndRotate.Rotate(currentGamepad1.left_trigger - currentGamepad1.right_trigger + currentGamepad2.left_trigger - currentGamepad2.right_trigger);
                slidesAndRotate.MoveSlide((currentGamepad1.dpad_up || currentGamepad2.dpad_up) ? 0.8 : 0 + ((currentGamepad1.dpad_down || currentGamepad2.dpad_down) ? -0.8 : 0));
            } else {
                slidesAndRotate.MoveSlide(currentPreset);
                slidesAndRotate.Rotate(currentPreset);
            }

            // Do the drivetrain. Left bumper is slow mode, right bumper is reverse the robot (does it work?).
            // REMEMBER Y STICK IS REVERSED
            /*
            driveTrain.Drive(
                    currentGamepad1.right_bumper ?  currentGamepad1.right_stick_x  * (currentGamepad1.right_bumper ? 1 : 1)  :  currentGamepad1.left_stick_x  * (currentGamepad1.right_bumper ? 1 : 1),
                    currentGamepad1.right_bumper ? -currentGamepad1.right_stick_y  * (currentGamepad1.right_bumper ? -1 : 1) : -currentGamepad1.left_stick_y  * (currentGamepad1.right_bumper ? -1 : 1),
                    currentGamepad1.right_bumper ?  currentGamepad1.left_stick_x   * (currentGamepad1.right_bumper ? 1 : 1)  :  currentGamepad1.right_stick_x * (currentGamepad1.right_bumper ? 1 : 1),
                    currentGamepad1.right_bumper ? -currentGamepad1.left_stick_y   * (currentGamepad1.right_bumper ? -1 : 1) : -currentGamepad1.right_stick_y * (currentGamepad1.right_bumper ? -1 : 1),
                    currentGamepad1.left_bumper ? 0.5 : 1);
             */
            double speed = 1;
            if (currentGamepad1.left_bumper) speed*=0.4;
            if (currentGamepad1.right_bumper) speed*=0.8;
            if (currentGamepad1.right_bumper && currentGamepad1.left_bumper) speed = 0.2;

            double r = lastR;


            if (currentGamepad1.left_stick_x != 0 || currentGamepad1.right_stick_x != 0 || currentGamepad1.left_stick_y != 0 || currentGamepad1.triangle || currentGamepad1.cross || currentGamepad1.circle || currentGamepad1.square) {
                double shift = currentGamepad1.right_stick_button ? Math.PI/4 : 0;
                // gamepad 1 driving
                if (currentGamepad1.square) r=Math.PI/2 + shift;
                else if (currentGamepad1.triangle) r=shift;
                else if (currentGamepad1.circle) r=-Math.PI/2 + shift;
                else if (currentGamepad1.cross) r=Math.PI + shift;
                else r = pos.getHeading(AngleUnit.RADIANS) + (-Math.PI / 2.0)/2*currentGamepad1.right_stick_x;
                driveTrain.DriveFieldCentric(currentGamepad1.left_stick_x, -currentGamepad1.left_stick_y, r, pos.getHeading(AngleUnit.RADIANS), speed, telemetry);
            } else {
                // gamepad 2 driving
                double X = currentGamepad2.left_stick_x;
                double Y = -currentGamepad2.left_stick_y;
                double rX = currentGamepad2.right_stick_x;
                double rY = -currentGamepad2.right_stick_y;
                if (Math.sqrt(rX*rX + rY*rY) > 0.8) {
                    if (rX > 0) {
                        if (rY > 0) {
                            // first quadrant, -pi/2 < r < 0
                            r = Math.atan(-rX / rY);
                            telemetry.addLine("quadrant 1");
                        } else if (rY < 0) {
                            // fourth quadrant, -pi < r < -pi/2
                            r = -Math.PI / 2 - Math.atan(rY / (-rX));
                            telemetry.addLine("quadrant 4");
                        } else {
                            // positive x axis
                            r = -Math.PI / 2;
                        }
                    } else if (rX < 0) {
                        if (rY > 0) {
                            // second quadrant, 0 < r < pi/2
                            r = -Math.atan(rX / rY);
                            telemetry.addLine("quadrant 2");
                        } else if (rY < 0) {
                            // third quadrant, pi/2 < r < pi
                            r = Math.PI / 2 + Math.atan(rY / rX);
                            telemetry.addLine("quadrant 3");
                        } else {
                            // negative x axis
                            r = Math.PI / 2;
                        }
                    } else {
                        if (rY > 0) {
                            // positive y axis
                            r = 0;
                        } else if (rY < 0) {
                            // negative y axis
                            r = Math.PI;
                        } else {
                            // x and y are 0
                            // should never happen, but just in case
                            r = 0;
                        }
                    }
                }
                driveTrain.DriveFieldCentric(X, Y, r, pos.getHeading(AngleUnit.RADIANS), speed, telemetry);
            }

            lastR = r;
            telemetry.addData("r (degrees)", r*180/Math.PI);


            // claw open and close
            if ((currentGamepad1.dpad_right && !previousGamepad1.dpad_right) || (currentGamepad2.triangle && !previousGamepad2.triangle)) {
                if (claw.getPosition() != CustomServo.Position.close) {
                    claw.moveToPos(CustomServo.Position.close);
                } else if (claw.getPosition() != CustomServo.Position.open) {
                    claw.moveToPos(CustomServo.Position.open);
                }
            }

            // claw mid
            if (currentGamepad1.dpad_left && !currentGamepad1.dpad_up && !currentGamepad1.dpad_down ||
                    currentGamepad2.dpad_left && !currentGamepad2.dpad_up && !currentGamepad2.dpad_down) {
                if (claw.getPosition() != CustomServo.Position.mid) {
                    claw.moveToPos(CustomServo.Position.mid);
                }
            }

            // clawRotate
            if (currentGamepad2.left_bumper) clawRotate.move(LastFrameTime, true);
            if (currentGamepad2.right_bumper) clawRotate.move(LastFrameTime, false);
            if (currentGamepad1.share || currentGamepad2.share) clawRotate.moveToPos(CustomServo.Position.mid);


            // do telemetry
            telemetry.addData("MS update Time", TimeElapsed);
            telemetry.addData("SlideRotate angle", slidesAndRotate.getAngle());
            telemetry.addData("Remaining Slide Distance", slidesAndRotate.GetRemainingSlideDistance());
            telemetry.addLine("Slide Encoder Difference: " + Math.abs(slidesAndRotate.slide.getCurrentPosition() - slidesAndRotate.slide2.getCurrentPosition()) + ", with values " + slidesAndRotate.slide.getCurrentPosition() + " and " + slidesAndRotate.slide2.getCurrentPosition());
            telemetry.addLine("SlideRotate Encoder Difference: " + Math.abs(slidesAndRotate.slideRotate.getCurrentPosition() - slidesAndRotate.slideRotate2.getCurrentPosition()) + ", with values " + slidesAndRotate.slideRotate.getCurrentPosition() + " and " + slidesAndRotate.slideRotate2.getCurrentPosition());
            telemetry.addData("Slide state", slidesAndRotate.getStateSlide());
            telemetry.addData("Rotate state", slidesAndRotate.getStateRotate());
            telemetry.addData("ClawRotate", clawRotate.getRawPosition());
            telemetry.addData("NEWWWW", true);
            telemetry.update();

        }
    }

}