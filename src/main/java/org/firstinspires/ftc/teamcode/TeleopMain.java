package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        SlidesAndRotate slidesAndRotate = new SlidesAndRotate(true, true);
        slidesAndRotate.initSlide(hardwareMap.get(DcMotor.class, "slide"),
                hardwareMap.get(DcMotor.class, "slide2"));
        slidesAndRotate.initRotate(hardwareMap.get(DcMotor.class, "slideRotate"),
                hardwareMap.get(DcMotor.class, "slideRotate2"));

        DriveTrain driveTrain = new DriveTrain();
        driveTrain.init(true,
                hardwareMap.get(DcMotor.class, "backLeft"),
                hardwareMap.get(DcMotor.class, "backRight"),
                hardwareMap.get(DcMotor.class, "frontLeft"),
                hardwareMap.get(DcMotor.class, "frontRight"));

        CustomServo claw = new CustomServo(0, 1);
        claw.init(hardwareMap.get(Servo.class, "claw"), CustomServo.Position.close);

        waitForStart();

        long LastFrameTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            slidesAndRotate.Rotate(gamepad1.left_trigger - gamepad1.right_trigger + gamepad2.left_trigger - gamepad2.right_trigger);
            slidesAndRotate.MoveSlide((gamepad1.dpad_up || gamepad2.dpad_up) ? 0.8 : 0 + ((gamepad1.dpad_down || gamepad2.dpad_down) ? -0.8 : 0));
            // Do the drivetrain. Left bumper is slow mode, right bumper is reverse the robot.
            driveTrain.Drive(
                    gamepad1.right_bumper ?  gamepad1.right_stick_x  * (gamepad1.right_bumper ? 1 : 1)  :  gamepad1.left_stick_x  * (gamepad1.right_bumper ? 1 : 1),
                    gamepad1.right_bumper ? -gamepad1.right_stick_y  * (gamepad1.right_bumper ? -1 : 1) : -gamepad1.left_stick_y  * (gamepad1.right_bumper ? -1 : 1),
                    gamepad1.right_bumper ?  gamepad1.left_stick_x   * (gamepad1.right_bumper ? 1 : 1)  :  gamepad1.right_stick_x * (gamepad1.right_bumper ? 1 : 1),
                    gamepad1.right_bumper ? -gamepad1.left_stick_y   * (gamepad1.right_bumper ? -1 : 1) : -gamepad1.right_stick_y * (gamepad1.right_bumper ? -1 : 1),
                    gamepad1.left_bumper ? 0.5 : 1);

            double TimeElapsed = System.currentTimeMillis() - LastFrameTime;
            LastFrameTime = System.currentTimeMillis();
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
