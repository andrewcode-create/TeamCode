package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "AutoMain")
public class AutoMain extends LinearOpMode {

    enum state2 {
        start,

        bar,
        goBarGround,
        goGround,
        ground,
        specimenPickup,

        done,

        lowerSlide,
        openClaw,
        closeClaw,
    }

    enum state {
        start,
        bar1, goBarGround1, goGround1, ground1,
        specimenPickup2, bar2, goBarGround2, goGround2, ground2,
        getSpecimenPickup3, bar3, goBarGround3, goGround3, ground3,
        getSpecimenPickup4, bar4,
        getSpecimenPickup5, bar5,
        done,

        lowerSlide,
        openClaw,
        closeClaw,

        broken,
    }

    enum mainState {
        ground1,
        ground2,
        ground3,
        pickup,
        drop1,
        drop2,
        drop3,
        drop4,
        drop5,
    }

    enum subState {
        closeClaw,
        openClaw,
        lowerSlide,
        alt1,
        alt2,
        alt3,
    }

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
        CustomServo claw = new CustomServo(0.7, 0.43);
        CustomServo clawRotate = new CustomServo(0, 1, 0.475, 0.004);
        claw.init(hardwareMap.get(Servo.class, "claw"), CustomServo.Position.close);
        clawRotate.init(hardwareMap.get(Servo.class, "clawRotate"), CustomServo.Position.mid);

        // set manual sensor caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // set telemetry to update the driver station 10 times per second
        telemetry.setMsTransmissionInterval(100);

        // odometer start
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(Constants.pinpointRotX, Constants.pinpointRotY);



        odo.resetPosAndIMU();
        telemetry.addLine("Waiting for odometer");
        telemetry.update();
        sleep(500);
        // set position to starting teleop position and heading
        odo.setPosition(new Pose2D(DistanceUnit.MM, Constants.startingPosAuto.getX(DistanceUnit.MM), Constants.startingPosAuto.getY(DistanceUnit.MM), AngleUnit.RADIANS, Constants.startingPosAuto.getHeading(AngleUnit.RADIANS)));
        sleep(20);


        // Ready!
        telemetry.addLine("Ready to play, everything initialized.");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Started");
        telemetry.update();

        long LastFrameTime = System.currentTimeMillis();
        long currentStateStartTime = System.currentTimeMillis();

        //state2 currentState = state2.start;
        //state2 previousState = state2.start;
        mainState currentMainState = mainState.drop5;
        subState currentSubState = subState.alt1;

        //int hungSpecimenNumber = 0;
        //int groundSpecimensTaken = 0;

        int stepnum = 1;
        int substepnum = 1;

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

            Pose2D vel = odo.getVelocity();

            boolean isStopped = vel.getX(DistanceUnit.MM) < 1 && vel.getY(DistanceUnit.MM) < 1 && vel.getHeading(AngleUnit.DEGREES) < 2;

            long currentStateTimeElapsed = System.currentTimeMillis() - currentStateStartTime;

            if (stepnum == 1) {
                // put on high bar, position 5
                if (substepnum == 1) {
                    // go to high bar, position 5
                    driveTrain.DriveToPoint(Constants.Drop5, pos, 1);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.TopSpecimen);
                    if (isStopped && currentStateTimeElapsed > 500) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 2) {
                    // lower linear slide
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.DropTopSpecimen);
                    if (currentStateTimeElapsed > 500) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 3) {
                    // open claw
                    claw.moveToPos(CustomServo.Position.open);
                    if (currentStateTimeElapsed > 500) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else  if (substepnum == 4) {
                    // go back 160 mm
                    Pose2D toPoint = new Pose2D(DistanceUnit.MM, Constants.Drop5.getX(DistanceUnit.MM) - 160, Constants.Drop5.getY(DistanceUnit.MM), AngleUnit.DEGREES, Constants.Drop5.getHeading(AngleUnit.DEGREES));
                    driveTrain.DriveToPointGoThrough(toPoint, pos, 1);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 20) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 2) {
                // go to x570, y930, r-90, stop
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 570, 930, AngleUnit.DEGREES, -90);
                driveTrain.DriveToPoint(toPoint, pos, 1);
                if (isStopped && currentStateTimeElapsed > 500) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 3) {
                // go to x810, y910, r-90, stop
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 810, 910, AngleUnit.DEGREES, -90);
                driveTrain.DriveToPoint(toPoint, pos, 1);
                if (isStopped && currentStateTimeElapsed > 500) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 4) {
                // go to x810, y752, r-90, stop
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 810, 750, AngleUnit.DEGREES, -90);
                driveTrain.DriveToPoint(toPoint, pos, 1);
                if (isStopped && currentStateTimeElapsed > 500) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 5) {
                // go to drop off ( x170 y760, r180 )
                driveTrain.DriveToPoint(Constants.pickUpSpecimen, pos, 1);
                if (isStopped && currentStateTimeElapsed > 500) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 6) {
                // go to x810, y250, r-90, stop
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 810, 250, AngleUnit.DEGREES, -90);
                driveTrain.DriveToPoint(toPoint, pos, 1);
                if (isStopped && currentStateTimeElapsed > 500) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 7) {
                // go to x400, y260, r-90, continue
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 400, 260, AngleUnit.DEGREES, -90);
                driveTrain.DriveToPointGoThrough(toPoint, pos, 1);
                if (DriveTrain.getDistanceToPoint(toPoint, pos) < 20) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 8) {
                // go to x150, y466, r180, stop & pick up
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 150, 466, AngleUnit.DEGREES, 180);
                driveTrain.DriveToPoint(toPoint, pos, 1);
                if (isStopped && currentStateTimeElapsed > 500) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 9) {
                // go to x400, y466, r180, continue
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 400, 466, AngleUnit.DEGREES, 180);
                driveTrain.DriveToPointGoThrough(toPoint, pos, 1);
                if (DriveTrain.getDistanceToPoint(toPoint, pos) < 20) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum >= 10 && stepnum <= 13) {
                // go to 160mm behind high bar, continue, put on high bar, go back 160 mm, continue
                // then go to pick up
                // this repeats!
                Pose2D bar = Constants.Drop4;
                if (stepnum == 11) bar = Constants.Drop3;
                if (stepnum == 12) bar = Constants.Drop2;
                if (stepnum == 13) bar = Constants.Drop1;

                if (stepnum == 1) {
                    // go to 160 mm behind bar, continue
                    Pose2D toPoint = new Pose2D(DistanceUnit.MM, bar.getX(DistanceUnit.MM) - 160, bar.getY(DistanceUnit.MM), AngleUnit.DEGREES, bar.getHeading(AngleUnit.DEGREES));
                    driveTrain.DriveToPointGoThrough(toPoint, pos, 1);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.TopSpecimen);
                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 20) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 2) {
                    // go to high bar
                    driveTrain.DriveToPoint(bar, pos, 1);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.TopSpecimen);
                    if (isStopped && currentStateTimeElapsed > 500) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 3) {
                    // lower linear slide
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.DropTopSpecimen);
                    if (currentStateTimeElapsed > 500) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 4) {
                    // open claw
                    claw.moveToPos(CustomServo.Position.open);
                    if (currentStateTimeElapsed > 500) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else  if (substepnum == 5) {
                    // go back 160 mm
                    Pose2D toPoint = new Pose2D(DistanceUnit.MM, bar.getX(DistanceUnit.MM) - 160, bar.getY(DistanceUnit.MM), AngleUnit.DEGREES, bar.getHeading(AngleUnit.DEGREES));
                    driveTrain.DriveToPointGoThrough(toPoint, pos, 1);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 20) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 6) {
                    // go to pick up
                    driveTrain.DriveToPoint(Constants.pickUpSpecimen, pos, 1);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    if (isStopped && currentStateTimeElapsed > 500) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 7) {
                    // close claw
                    claw.moveToPos(CustomServo.Position.close);
                    if (currentStateTimeElapsed > 500) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else {
                // done
            }

            // do telemetry
            telemetry.addData("MS update Time", TimeElapsed);
            telemetry.addData("SlideRotate angle", slidesAndRotate.getAngle());
            telemetry.addData("Remaining Slide Distance", slidesAndRotate.GetRemainingSlideDistance());
            telemetry.addLine("Slide Encoder Difference: " + Math.abs(slidesAndRotate.slide.getCurrentPosition() - slidesAndRotate.slide2.getCurrentPosition()) + ", with values " + slidesAndRotate.slide.getCurrentPosition() + " and " + slidesAndRotate.slide2.getCurrentPosition());
            telemetry.addLine("SlideRotate Encoder Difference: " + Math.abs(slidesAndRotate.slideRotate.getCurrentPosition() - slidesAndRotate.slideRotate2.getCurrentPosition()) + ", with values " + slidesAndRotate.slideRotate.getCurrentPosition() + " and " + slidesAndRotate.slideRotate2.getCurrentPosition());
            telemetry.addData("Slide state", slidesAndRotate.getStateSlide());
            telemetry.addData("Rotate state", slidesAndRotate.getStateRotate());
            telemetry.addLine();
            telemetry.addLine("auto stepnum: " + stepnum + ", substepnum: " + substepnum);
            telemetry.update();
        }
    }

}
