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
                //put on high bar, position 5
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
            }


            /*
            if (currentMainState == mainState.drop5) {

            }
            */


            /*
            switch (currentState) {
                case start:
                    previousState = currentState;
                    currentState = state2.bar;
                    currentStateStartTime = System.currentTimeMillis();
                    break;
                case bar:
                    Pose2D goTo;
                    if (hungSpecimenNumber == 0) goTo = Constants.Drop1;
                    else if (hungSpecimenNumber == 1) goTo = Constants.Drop2;
                    else if (hungSpecimenNumber == 2) goTo = Constants.Drop3;
                    else if (hungSpecimenNumber == 3) goTo = Constants.Drop4;
                    else goTo = Constants.Drop5; // hungSpecimenNumber == 4
                    driveTrain.DriveToPoint(goTo, pos, 1);
                    if (currentStateTimeElapsed > 300) {
                        slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.TopSpecimen);
                    }
                    if (isStopped) {
                        previousState = currentState;
                        currentState = state2.lowerSlide;
                        currentStateStartTime = System.currentTimeMillis();
                    }
                    break;
                case lowerSlide:
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.DropTopSpecimen);
                    if (currentStateTimeElapsed > 500) {
                        previousState = currentState;
                        currentState = state2.openClaw;
                        currentStateStartTime = System.currentTimeMillis();
                    }
                    break;
                case openClaw:
                    claw.moveToPos(CustomServo.Position.open);
                    if (currentStateTimeElapsed > 500) {
                        previousState = currentState;
                        currentStateStartTime = System.currentTimeMillis();
                        hungSpecimenNumber++;
                        if (groundSpecimensTaken < 3) {
                            currentState = state2.goBarGround;
                        } else if (hungSpecimenNumber < 5){
                            currentState = state2.specimenPickup;
                        } else {
                            currentState = state2.done;
                        }
                    }
                    break;
                case goBarGround:
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    driveTrain.DriveToPointGoThrough(Constants.goBarGround, pos, 1);
                    if (DriveTrain.getDistanceToPoint(pos,Constants.goBarGround) < 20) {
                        previousState = currentState;
                        currentStateStartTime = System.currentTimeMillis();
                        currentState = state2.goGround;
                    }
                    break;
                case specimenPickup:
                    driveTrain.DriveToPoint(Constants.pickUpSpecimen, pos, 1);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    if (isStopped) {
                        previousState = currentState;
                        currentStateStartTime = System.currentTimeMillis();
                        currentState = state2.closeClaw;
                    }
                    break;
                case goGround:
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    if (groundSpecimensTaken == 0) {
                        driveTrain.DriveToPoint(Constants.goGround1, pos, 1);
                    } else if (groundSpecimensTaken == 1) {
                        driveTrain.DriveToPoint(Constants.goGround2, pos, 1);
                    } else if (groundSpecimensTaken == 2) {
                        driveTrain.DriveToPoint(Constants.goGround3, pos, 1);
                    }
                    if (isStopped) {
                        previousState = currentState;
                        currentStateStartTime = System.currentTimeMillis();
                        currentState = state2.ground;
                    }
                    break;
                case closeClaw:
                    claw.moveToPos(CustomServo.Position.close);
                    if (currentStateTimeElapsed > 500) {
                        previousState = currentState;
                        currentStateStartTime = System.currentTimeMillis();
                        currentState = state2.specimenPickup;
                    }
                    break;
                case done:
                    driveTrain.DriveToPoint(Constants.pickUpSpecimen, pos, 1);
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.WallPickup);
                    break;
                case ground:
                    Pose2D point = null;
                    if (groundSpecimensTaken == 0) {
                        driveTrain.DriveToPointGoThrough(Constants.ground1, pos, 1);
                        point = Constants.ground1;
                    } else if (groundSpecimensTaken == 1) {
                        driveTrain.DriveToPointGoThrough(Constants.ground2, pos, 1);
                        point = Constants.ground2;
                    } else if (groundSpecimensTaken == 2) {
                        driveTrain.DriveToPointGoThrough(Constants.ground3, pos, 1);
                        point = Constants.ground2;
                    }
                    if (DriveTrain.getDistanceToPoint(point, pos) < 20) {
                        groundSpecimensTaken++;
                        previousState = currentState;
                        currentStateStartTime = System.currentTimeMillis();
                        currentState = state2.specimenPickup;
                    }
                    break;
            }
            */
            /*
            switch (currentState) {
                case start:
                    previousState = currentState;
                    currentState = state.bar1;
                    currentStateStartTime = System.currentTimeMillis();
                    break;
                case lowerSlide:
                    slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.DropTopSpecimen);
                    if (currentStateTimeElapsed > 500) {
                        previousState = currentState;
                        currentState = state.openClaw;
                        currentStateStartTime = System.currentTimeMillis();
                        specimenNumber++;
                    }
                    break;
                case openClaw:
                    claw.moveToPos(CustomServo.Position.open);
                    if (currentStateTimeElapsed > 500) {
                        previousState = currentState;
                        currentStateStartTime = System.currentTimeMillis();
                        switch(specimenNumber) {
                            case 2:
                                currentState = state.goBarGround1;
                                break;
                            case 3:
                                currentState = state.goBarGround2;
                                break;
                            case 4:
                                currentState = state.goBarGround3;
                                break;
                            case 5:
                                currentState = state.getSpecimenPickup4;
                                break;
                            case 6:
                                currentState = state.getSpecimenPickup5;
                                break;
                            case 7:
                                currentState = state.done;
                                break;
                            default:
                                currentState = state.broken;
                        }
                    }
                    break;
                case bar1:
                    driveTrain.DriveToPoint(Constants.Drop1, pos, 1);
                    if (currentStateTimeElapsed > 300) {
                        slidesAndRotate.MoveSlide(SlidesAndRotate.Presets.TopSpecimen);
                    }
                    if (isStopped)  {
                        previousState = currentState;
                        currentState = state.lowerSlide;
                        currentStateStartTime = System.currentTimeMillis();
                    }
                    break;
            }
            */



            /*
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
            */

            /*
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
             */

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

            /*
            double X = currentGamepad1.left_stick_x;
            double Y = -currentGamepad1.left_stick_y;
            double rX = currentGamepad1.right_stick_x;
            double rY = -currentGamepad1.right_stick_y;
            double r;
            if (rX > 0) {
                if (rY > 0) {
                    // first quadrant, -pi/2 < r < 0
                    r = -Math.atan(-rX/rY);
                } else if (rY < 0) {
                    // fourth quadrant, -pi < r < -pi/2
                    r = -Math.PI/2 - Math.atan(rX/(-rY));
                } else {
                    // positive x axis
                    r = -Math.PI/2;
                }
            } else if (rX < 0) {
                if (rY > 0) {
                    // second quadrant, 0 < r < pi/2
                    r = Math.atan(rX/rY);
                } else if (rY < 0) {
                    // third quadrant, pi/2 < r < pi
                    r = Math.PI/2 + Math.atan(rX/rY);
                } else {
                    // negative x axis
                    r = Math.PI/2;
                }
            } else {
                if (rY > 0) {
                    // positive y axis
                    r = 0;
                } else {
                    // negative y axis
                    r = Math.PI;
                }
            }
            */

            //double r = (currentGamepad1.right_stick_x > 0 ? -1 : 1)*(Math.PI*0.5 + Math.atan(-currentGamepad1.right_stick_y / (currentGamepad1.right_stick_x == 0 ? 0.001 : currentGamepad1.right_stick_x)));
            //driveTrain.DriveFieldCentric(X, Y, r, pos.getHeading(AngleUnit.RADIANS),1, telemetry);

            /*
            // claw
            if (currentGamepad1.y && !previousGamepad1.y && claw.getPosition() != CustomServo.Position.close) {
                claw.moveToPos(CustomServo.Position.close);
            } else if (currentGamepad1.y && !previousGamepad1.y && claw.getPosition() != CustomServo.Position.open) {
                claw.moveToPos(CustomServo.Position.open);
            }
            */


            // do telemetry
            telemetry.addData("MS update Time", TimeElapsed);
            telemetry.addData("SlideRotate angle", slidesAndRotate.getAngle());
            telemetry.addData("Remaining Slide Distance", slidesAndRotate.GetRemainingSlideDistance());
            telemetry.addLine("Slide Encoder Difference: " + Math.abs(slidesAndRotate.slide.getCurrentPosition() - slidesAndRotate.slide2.getCurrentPosition()) + ", with values " + slidesAndRotate.slide.getCurrentPosition() + " and " + slidesAndRotate.slide2.getCurrentPosition());
            telemetry.addLine("SlideRotate Encoder Difference: " + Math.abs(slidesAndRotate.slideRotate.getCurrentPosition() - slidesAndRotate.slideRotate2.getCurrentPosition()) + ", with values " + slidesAndRotate.slideRotate.getCurrentPosition() + " and " + slidesAndRotate.slideRotate2.getCurrentPosition());
            telemetry.addData("Slide state", slidesAndRotate.getStateSlide());
            telemetry.addData("Rotate state", slidesAndRotate.getStateRotate());
            telemetry.addLine();
            telemetry.addLine("auto state: " + currentState.name() + ", previous state: " + previousState.name());
            telemetry.update();


        }
    }

}
