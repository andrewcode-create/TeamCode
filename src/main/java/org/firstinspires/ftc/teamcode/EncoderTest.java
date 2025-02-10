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

@TeleOp(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {


        // init drivetrain
        DriveTrain driveTrain = new DriveTrain();

        DcMotorEx[] blah = { hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")};
        driveTrain.init(true,
                blah[0],
                blah[1],
                blah[2],
                blah[3]);

        DcMotor slideLeft =  hardwareMap.get(DcMotor.class, "slideLeft");
        DcMotor slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        for (DcMotor m : blah) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }



        // set manual sensor caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }



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



        odo.resetPosAndIMU();
        telemetry.addLine("Waiting for odometer");
        telemetry.update();
        sleep(500);
        // set position to starting teleop position and heading
        odo.setPosition(Constants.startingPosAutoSample);
        sleep(20);


        // Ready!
        telemetry.addLine("Ready to play, everything initialized.");
        telemetry.update();
        waitForStart();

        double lastR = 0;

        long LastFrameTime = System.currentTimeMillis();
        boolean pressed = false;
        while (opModeIsActive()) {
            telemetry.addLine("WOOOOOOO222222222!");
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

            if (gamepad1.square || gamepad2.square) {
                for (DcMotor m : blah) {
                    m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                pressed = true;

            } else if (gamepad1.triangle || gamepad2.triangle) {
                pressed = false;
                for (DcMotor m : blah) {
                    m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }

            telemetry.addData("Pressed", pressed);

            if (pressed) {
                driveTrain.DriveToPoint(Constants.Drop5, pos, 1);
            }


            // do telemetry
            telemetry.addData("MS update Time", TimeElapsed);
            telemetry.addData("SlideLeft", slideLeft.getCurrentPosition());
            telemetry.addData("SlideRight", slideRight.getCurrentPosition());
            telemetry.update();


        }
    }

}
