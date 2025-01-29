package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
                hardwareMap.get(DcMotor.class, "slideRight"));
        slidesAndRotate.initRotate(hardwareMap.get(DcMotor.class, "slideRotateLeft"),
                hardwareMap.get(DcMotor.class, "slideRotateRight"));
        SlidesAndRotate.Presets currentPreset = null;

        // init drivetrain
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.init(true,
                hardwareMap.get(DcMotor.class, "backLeft"),
                hardwareMap.get(DcMotor.class, "backRight"),
                hardwareMap.get(DcMotor.class, "frontLeft"),
                hardwareMap.get(DcMotor.class, "frontRight"));

        // init claw
        CustomServo claw = new CustomServo(0.71, 0.85);
        claw.init(hardwareMap.get(Servo.class, "claw"), CustomServo.Position.close);

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
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        telemetry.addLine("Waiting for odometer");
        telemetry.update();
        sleep(500);


        // Ready!
        telemetry.addLine("Ready to play, everything initialized.");
        telemetry.update();
        waitForStart();

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

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            // Store the gamepad values from the previous loop iteration in previous gamepad1/2 to be used in this loop iteration.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being used in the same loop iteration.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // do sync if necessary
            if (currentGamepad1.share && !previousGamepad1.share || currentGamepad2.share && !previousGamepad2.share) {
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
            } else if (currentGamepad1.a && !previousGamepad1.a || currentGamepad2.a && !previousGamepad2.a) {
                currentPreset = SlidesAndRotate.Presets.WallPickup;
            } else if (currentGamepad1.b && !previousGamepad1.b || currentGamepad2.b && !previousGamepad2.b) {
                currentPreset = SlidesAndRotate.Presets.TopSpecimen;
            } else if (currentGamepad1.guide && !previousGamepad1.guide || currentGamepad2.guide && !previousGamepad2.guide) {
                currentPreset = SlidesAndRotate.Presets.Ascent;
            }

            // do slide rotation and extension
            if (currentPreset == null) {
                slidesAndRotate.Rotate(currentGamepad1.left_trigger - currentGamepad1.right_trigger + currentGamepad2.left_trigger - currentGamepad2.right_trigger);
                slidesAndRotate.MoveSlide((currentGamepad1.dpad_up || currentGamepad2.dpad_up) ? 0.8 : 0 + ((currentGamepad1.dpad_down || currentGamepad2.dpad_down) ? -0.8 : 0));
            } else {
                slidesAndRotate.Rotate(currentPreset);
                slidesAndRotate.MoveSlide(currentPreset);
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
            double X = currentGamepad1.left_stick_x;
            double Y = -currentGamepad1.left_stick_y;
            double rX = currentGamepad1.right_stick_x;
            double rY = -currentGamepad1.right_stick_y;
            double r;
            if (rY < 0) {
                r = Math.atan(rY/rX);
            } else if (rY > 0) {
                r = -Math.atan(rY/rX);
            } else {
                if (rX>=0) {
                    r = 0;
                } else {
                    r = Math.PI;
                }
            }
            //double r = (currentGamepad1.right_stick_x > 0 ? -1 : 1)*(Math.PI*0.5 + Math.atan(-currentGamepad1.right_stick_y / (currentGamepad1.right_stick_x == 0 ? 0.001 : currentGamepad1.right_stick_x)));
            driveTrain.DriveFieldCentric(X, Y, r, pos.getHeading(AngleUnit.RADIANS),1, telemetry);


            // claw
            if (currentGamepad1.y && !previousGamepad1.y && claw.getPosition() != CustomServo.Position.close) {
                claw.moveToPos(CustomServo.Position.close);
            } else if (currentGamepad1.y && !previousGamepad1.y && claw.getPosition() != CustomServo.Position.open) {
                claw.moveToPos(CustomServo.Position.open);
            }


            // do telemetry
            telemetry.addData("MS update Time", TimeElapsed);
            telemetry.addData("SlideRotate angle", slidesAndRotate.getAngle());
            telemetry.addData("Remaining Slide Distance", slidesAndRotate.GetRemainingSlideDistance());
            telemetry.addLine("Slide Encoder Difference: " + Math.abs(slidesAndRotate.slide.getCurrentPosition() - slidesAndRotate.slide2.getCurrentPosition()) + ", with values " + slidesAndRotate.slide.getCurrentPosition() + " and " + slidesAndRotate.slide2.getCurrentPosition());
            telemetry.addLine("Slide Encoder Difference: " + Math.abs(slidesAndRotate.slideRotate.getCurrentPosition() - slidesAndRotate.slideRotate2.getCurrentPosition()) + ", with values " + slidesAndRotate.slideRotate.getCurrentPosition() + " and " + slidesAndRotate.slideRotate2.getCurrentPosition());
            telemetry.addData("Slide state", slidesAndRotate.getStateSlide());
            telemetry.addData("Rotate state", slidesAndRotate.getStateRotate());
            telemetry.update();


        }
    }

}
