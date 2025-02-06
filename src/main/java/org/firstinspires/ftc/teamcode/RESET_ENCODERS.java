package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name = "RESET ENCODERS")
public class RESET_ENCODERS extends LinearOpMode {

    @Override
    public void runOpMode() {
        SlidesAndRotate slidesAndRotate = new SlidesAndRotate(true, true);

        DriveTrain driveTrain = new DriveTrain();
        driveTrain.init(true,
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"));

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(Constants.pinpointRotX, Constants.pinpointRotY);

        boolean startedSlides = false;

        waitForStart();
        while(opModeIsActive()) {
            driveTrain.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad2.right_stick_x, -gamepad2.right_stick_y, 0.5);

            if (gamepad1.square) {
                telemetry.addLine("Resetting odometry");
                telemetry.update();
                odo.resetPosAndIMU();
                sleep(500);
            }
            if (gamepad1.triangle) {
                slidesAndRotate.initSlide(hardwareMap.get(DcMotor.class, "slideLeft"),
                        hardwareMap.get(DcMotor.class, "slideRight"), true);
                slidesAndRotate.initRotate(hardwareMap.get(DcMotor.class, "slideRotateLeft"),
                        hardwareMap.get(DcMotor.class, "slideRotateRight"), true);
                startedSlides = true;
            }

            if (startedSlides) {
                slidesAndRotate.RotateRaw(0.5*(gamepad1.left_trigger - gamepad1.right_trigger));
                slidesAndRotate.MoveSlideRaw(0.6*((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0)));
            }

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.1f, Y: %.1f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("odo", data);
            telemetry.addLine("Use square button to reset odometry");
            telemetry.addLine("Use triangle button to" + (startedSlides ? "RESET" : "ENABLE") + "slides and slideRotate");
            telemetry.update();
        }
    }
}


