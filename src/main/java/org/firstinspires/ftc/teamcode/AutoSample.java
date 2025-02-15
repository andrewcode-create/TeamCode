package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "AutoSample", preselectTeleOp="TeleopMain")
public class AutoSample extends LinearOpMode {

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
                hardwareMap.get(DcMotor.class, "slideRight"), true);
        slidesAndRotate.initRotate(hardwareMap.get(DcMotor.class, "slideRotateLeft"),
                hardwareMap.get(DcMotor.class, "slideRotateRight"), true);
        SlidesAndRotate.Presets currentPreset = null;

        // init drivetrain
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.init(true,
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"));

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
        odo.setPosition(new Pose2D(DistanceUnit.MM, Constants.startingPosAutoSample.getX(DistanceUnit.MM), Constants.startingPosAutoSample.getY(DistanceUnit.MM), AngleUnit.RADIANS, Constants.startingPosAutoSample.getHeading(AngleUnit.RADIANS)));
        sleep(20);


        // Ready!
        telemetry.addLine("Ready to play, everything initialized.");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Started");
        telemetry.update();

        long LastFrameTime = System.currentTimeMillis();
        long currentStateStartTime = System.currentTimeMillis();
        long otherStateStartTime = System.currentTimeMillis();

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

            boolean isStopped = vel.getX(DistanceUnit.MM) < 0.3 && vel.getY(DistanceUnit.MM) < 0.3 && vel.getHeading(AngleUnit.DEGREES) < 2;
            boolean isStoppedTurn = vel.getHeading(AngleUnit.DEGREES) < 2 && vel.getHeading(AngleUnit.DEGREES) > -2;

            long currentStateTimeElapsed = System.currentTimeMillis() - currentStateStartTime;
            long otherStateTimeElapsed = System.currentTimeMillis() - otherStateStartTime;

            if (stepnum == 1) {
                // put in low basket
                if (substepnum == 1) {
                    // go to low basket
                    Pose2D toPoint = Constants.Basket;
                    driveTrain.DriveToPoint(toPoint, pos, 0.5);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.LowBasket);
                    slidesAndRotate.Rotate(SlidesAndRotate.Presets.LowBasket);
                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 30 && currentStateTimeElapsed > 1000) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 2) {
                    // open claw
                    claw.moveToPos(CustomServo.Position.open);
                    driveTrain.Drive(0,0,0,0,0);
                    if (currentStateTimeElapsed > 300) {
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
                // go to first on ground
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 550, 4100, AngleUnit.DEGREES, 0);
                if (DriveTrain.getDistanceToPoint(toPoint, pos) > 60) {
                    driveTrain.DriveToPoint(toPoint, pos, 0.9);
                    otherStateStartTime = System.currentTimeMillis();
                } else {
                    driveTrain.DriveToPoint(toPoint, pos, 0.5);
                }
                slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                slidesAndRotate.Rotate(SlidesAndRotate.Presets.WallPickup);
                if (DriveTrain.getDistanceToPoint(toPoint, pos) < 5 && currentStateTimeElapsed > 1000 && otherStateTimeElapsed > 1500) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 3) {
                if (substepnum == 1) {
                    // pick up first on ground
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.FloorPickup);
                    slidesAndRotate.Rotate(SlidesAndRotate.Presets.FloorPickup);
                    driveTrain.stopMotors();
                    if (currentStateTimeElapsed > 2000) {
                        // done, move to next step
                        substepnum++;
                        currentStateStartTime = System.currentTimeMillis();
                    }
                } else {
                    // close claw
                    claw.moveToPos(CustomServo.Position.close);
                    driveTrain.stopMotors();
                    if (currentStateTimeElapsed > 600) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum = 1;
                        stepnum++;
                    }
                }
            } else if (stepnum == 4) {
                // put first in low basket
                if (substepnum == 1) {
                    // go to low basket
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.LowBasket);
                    slidesAndRotate.Rotate(SlidesAndRotate.Presets.LowBasket);
                    Pose2D toPoint;

                    if (currentStateTimeElapsed > 1000) {
                        if (pos.getHeading(AngleUnit.DEGREES) > -90 && pos.getHeading(AngleUnit.DEGREES) < 90) {
                            toPoint = new Pose2D(DistanceUnit.MM, Constants.Basket.getX(DistanceUnit.MM), Constants.Basket.getY(DistanceUnit.MM), AngleUnit.DEGREES, -170);
                        } else {
                            toPoint = new Pose2D(DistanceUnit.MM, Constants.Basket.getX(DistanceUnit.MM), Constants.Basket.getY(DistanceUnit.MM), AngleUnit.DEGREES, Constants.Basket.getHeading(AngleUnit.DEGREES));
                        }
                        driveTrain.DriveToPoint(toPoint, pos, 0.6);
                    }
                    toPoint = Constants.Basket;
                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 30 && currentStateTimeElapsed > 1000 && isStoppedTurn) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 2) {
                    // open claw
                    claw.moveToPos(CustomServo.Position.open);
                    driveTrain.stopMotors();
                    if (currentStateTimeElapsed > 300) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 5) {
                // go to second on ground
                Pose2D toPoint = new Pose2D(DistanceUnit.MM, 550, 4360, AngleUnit.DEGREES, 0);
                if (DriveTrain.getDistanceToPoint(toPoint, pos) > 60) {
                    driveTrain.DriveToPoint(toPoint, pos, 0.9);
                    otherStateStartTime = System.currentTimeMillis();
                } else {
                    driveTrain.DriveToPoint(toPoint, pos, 0.5);
                }
                slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                slidesAndRotate.Rotate(SlidesAndRotate.Presets.WallPickup);
                if (DriveTrain.getDistanceToPoint(toPoint, pos) < 5 && currentStateTimeElapsed > 500 && otherStateTimeElapsed > 1500) {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 6) {
                if (substepnum == 1) {
                    // pick up second on ground
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.FloorPickup);
                    slidesAndRotate.Rotate(SlidesAndRotate.Presets.FloorPickup);
                    driveTrain.stopMotors();
                    if (currentStateTimeElapsed > 2000) {
                        // done, move to next step
                        substepnum++;
                        currentStateStartTime = System.currentTimeMillis();
                    }
                } else {
                    // close claw
                    claw.moveToPos(CustomServo.Position.close);
                    driveTrain.Drive(0,0,0,0,0);
                    if (currentStateTimeElapsed > 600) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum = 1;
                        stepnum++;
                    }
                }
            } else if (stepnum == 7) {
                // put second in low basket
                if (substepnum == 1) {
                    // go to low basket
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.LowBasket);
                    slidesAndRotate.Rotate(SlidesAndRotate.Presets.LowBasket);
                    Pose2D toPoint;

                    if (currentStateTimeElapsed > 1000) {
                        if (pos.getHeading(AngleUnit.DEGREES) > -90 && pos.getHeading(AngleUnit.DEGREES) < 90) {
                            toPoint = new Pose2D(DistanceUnit.MM, Constants.Basket.getX(DistanceUnit.MM), Constants.Basket.getY(DistanceUnit.MM), AngleUnit.DEGREES, -170);
                        } else {
                            toPoint = Constants.Basket;
                            //toPoint = new Pose2D(DistanceUnit.MM, Constants.Basket.getX(DistanceUnit.MM), Constants.Basket.getY(DistanceUnit.MM), AngleUnit.DEGREES, Constants.Basket.getHeading(AngleUnit.DEGREES));
                        }
                        driveTrain.DriveToPoint(toPoint, pos, 0.6);
                    }
                    toPoint = Constants.Basket;


                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 30 && currentStateTimeElapsed > 500 && isStoppedTurn) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else if (substepnum == 2) {
                    // open claw
                    claw.moveToPos(CustomServo.Position.open);
                    driveTrain.stopMotors();
                    if (currentStateTimeElapsed > 300) {
                        currentStateStartTime = System.currentTimeMillis();
                        substepnum++;
                    }
                } else {
                    // done, move to next step
                    stepnum++;
                    substepnum = 1;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else if (stepnum == 8) {
                // go to x800, y3000, h180 through x800, y2500, h180 to push third into sample area.
                // then go to x200 y3000 h180
                if (substepnum == 1) {
                    Pose2D toPoint = new Pose2D(DistanceUnit.MM, 800, 4380, AngleUnit.DEGREES, 180);
                    driveTrain.DriveToPoint(toPoint, pos, 0.6);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    slidesAndRotate.Rotate(SlidesAndRotate.Presets.WallPickup);
                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 60 && currentStateTimeElapsed > 500) {
                        substepnum++;
                        currentStateStartTime = System.currentTimeMillis();
                    }
                } else if (substepnum == 2) {
                    Pose2D toPoint = new Pose2D(DistanceUnit.MM, 1267, 4650, AngleUnit.DEGREES, 180);
                    driveTrain.DriveToPoint(toPoint, pos, 0.6);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    slidesAndRotate.Rotate(SlidesAndRotate.Presets.WallPickup);
                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 40 && currentStateTimeElapsed > 500) {
                        substepnum++;
                        currentStateStartTime = System.currentTimeMillis();
                    }
                } else if (substepnum == 3) {
                    Pose2D toPoint = new Pose2D(DistanceUnit.MM, 270, 4650, AngleUnit.DEGREES, 180);
                    driveTrain.DriveToPoint(toPoint, pos, 0.6);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    slidesAndRotate.Rotate(SlidesAndRotate.Presets.WallPickup);
                    if (DriveTrain.getDistanceToPoint(toPoint, pos) < 40 && currentStateTimeElapsed > 500) {
                        substepnum++;
                        currentStateStartTime = System.currentTimeMillis();
                    }
                } else {
                    // done, go to next step
                    substepnum = 1;
                    stepnum++;
                    currentStateStartTime = System.currentTimeMillis();
                }
            } else {
                // Park
                Pose2D toPoint;
                if (currentStateTimeElapsed < 2000) {
                    toPoint = new Pose2D(DistanceUnit.MM, 1170, 4000, AngleUnit.DEGREES, -90);
                    driveTrain.DriveToPoint(toPoint, pos, 0.8);
                } else {
                    toPoint = new Pose2D(DistanceUnit.MM, 1170, 3550, AngleUnit.DEGREES, -90);
                    driveTrain.DriveToPoint(toPoint, pos, 0.8);
                }
                //driveTrain.DriveToPoint(toPoint, pos, 0.8);
                slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.ParkSample);
                slidesAndRotate.Rotate(SlidesAndRotate.Presets.ParkSample);
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
