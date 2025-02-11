package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlidesAndRotate {
    public DcMotor slide;
    public DcMotor slide2;
    public DcMotor slideRotate;
    public DcMotor slideRotate2;


    public final boolean VModeSlide;
    public final boolean VModeRotate;

    // class constructor
    public SlidesAndRotate(boolean VSlide, boolean VRotate) {
        this.VModeRotate = VRotate;
        this.VModeSlide = VSlide;
    }

    private State stateSlide = State.DriverControlled;
    private State stateRotate = State.DriverControlled;
    public State getStateRotate() {return stateRotate;};
    public State getStateSlide() {return stateSlide;};

    public enum State {
        DriverControlled,
        Preset,
        Ascend,
        ForcedRetract,
        Broken,
    }
    public enum Presets {
        WallPickup(0, 0, entryType.encoder),
        TopSpecimen(2550, 0, entryType.encoder),
        TopSpecimenXTraHigh(2650, 0, entryType.encoder),
        DropTopSpecimen(1700, 0, entryType.encoder),
        Middle(1125, 2200, entryType.encoder),
        LowBasket(3550, 250, entryType.encoder),
        Ascent(0, 0, entryType.encoder),
        FloorPickup(980, 2630, entryType.encoder),
        ParkSample(700, 500, entryType.encoder);

        // slide encoder value for the preset
        public final int slideEncoder;
        // slideRotate encoder value for the preset
        public final int slideRotateEncoder;

        private enum entryType {
            real, encoder
        }

        /*
         * @param slide - inches the slide should be off the ground.
         * @param slideRotate - degrees above horizontal the slide should be.
         */
        Presets(double slide, double slideRotate, entryType type) {
            if (type == entryType.encoder) {
                this.slideEncoder = (int)slide;
                this.slideRotateEncoder = (int)slideRotate;
            } else {
                // TODO proper conversion
                this.slideEncoder = (int) (slide * 100);
                this.slideRotateEncoder = (int) (slideRotate * 1000);
                throw new Error("TODO PROPER CONVERSION IN PRESET ENUM. ERROR!!!");

            }
        }
    }


    // Slide constants
    public final int RotateDownLim = 2800;
    public final int RotateUpLim = -100;
    public final int RotateVEncoder = 0;
    public final int RotateHEncoder = 2600;
    private final int RotateBuffer = 200;
    private final int RotateBufferUpDown = 100;

    public final int SlideMaxEncoder = 4080;
    public final double SlideMaxLength = 25.5;
    public final int SlideBufferDown = 200;
    public final int SlideBufferUp = 300;
    public final int SlideBuffer = 250;

    public final double SlidePositionInRobotFromBack = 11;
    public final double SlideAllowedOffsetInSlideCanGoPastBackOfRobot = 0;
    public final double SlideAllowedInSlideCanGoPastFrontOfRobot = 37;
    public final double SlideArmLength = 10.5;



    // Angle
    public double getAngle(int SlideRotate_Encoder) {
        return 90 * ((double)SlideRotate_Encoder / (RotateVEncoder - RotateHEncoder) + RotateHEncoder / (double)(RotateHEncoder - RotateVEncoder));
    }
    public double getAngle() {
        return getAngle((slideRotate.getCurrentPosition() + slideRotate2.getCurrentPosition())/2);
    }

    public void initSlide(DcMotor slideMotor, DcMotor slideMotor2, boolean RESET_ENCODER) {
        slide = slideMotor;
        slide2 = slideMotor2;
        if (RESET_ENCODER) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);
        if (VModeSlide) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void initRotate(DcMotor slideRotateMotor, DcMotor slideRotateMotor2, boolean RESET_ENCODER) {
        slideRotate = slideRotateMotor;
        slideRotate2 = slideRotateMotor2;
        if (RESET_ENCODER) {
            slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRotate2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotate2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotate.setDirection(DcMotor.Direction.REVERSE);
        slideRotate2.setDirection(DcMotor.Direction.REVERSE);
        if (VModeRotate) {
            slideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void RotateRaw(double power) {
        slideRotate.setPower(power);
        slideRotate2.setPower(power);
    }
    public void MoveSlideRaw(double power) {
        slide.setPower(power);
        slide2.setPower(power);
    }

    public void Rotate(Presets preset, double speed) {
        // check if encoders don't match, with buffer
        if (Math.abs(slideRotate.getCurrentPosition() - slideRotate2.getCurrentPosition()) > RotateBuffer) {
            stateRotate = State.Broken;
        } else if (stateRotate == State.Broken) {
            // state is broken, but within limits
            stateRotate = State.Preset;
        }

        // stop rotate motors if broken
        if (stateRotate == State.Broken) {
            // TODO proper logging
            slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRotate.setPower(0);
            slideRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRotate2.setPower(0);
            return;
        }
/*
        Presets preset1;
        if (preset.equals("WallPickup")) {
            preset1 = Presets.WallPickup;
        } else if (preset.equals("Ascent")) {
            preset1 = Presets.Ascent;
        } else if (preset.equals("TopSpecimen")) {
            preset1 = Presets.TopSpecimen;
        } else if (preset.equals("DropTopSpecimen")) {
            preset1 = Presets.DropTopSpecimen;
        } else {
            throw new Error("PRESET NOT VALID: " + preset);
        }*/

        // TODO do I need to check for if the position is reached, or not?
        stateRotate = State.Preset;
        slideRotate.setTargetPosition(preset.slideRotateEncoder);
        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotate.setPower(speed);
        slideRotate2.setTargetPosition(preset.slideRotateEncoder);
        slideRotate2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotate2.setPower(speed);
    }
    public void Rotate(Presets preset) {
        Rotate(preset, 0.8);
    }
    public void Rotate(double speed) {
        // check if encoders don't match, with buffer
        if (Math.abs(slideRotate.getCurrentPosition() - slideRotate2.getCurrentPosition()) > RotateBuffer) {
            stateRotate = State.Broken;
        } else if (stateRotate == State.Broken) {
            // state is broken, but within limits
            stateRotate = State.DriverControlled;
            if (VModeRotate) {
                slideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        // stop rotate motors if broken
        if (stateRotate == State.Broken) {
            // TODO proper logging
            slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRotate.setPower(0);
            slideRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRotate2.setPower(0);
            return;
        }

        // set mode if necessary
        if (stateRotate != State.DriverControlled) {
            if (VModeRotate) {
                slideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRotate2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideRotate2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (slideRotate.getCurrentPosition() > RotateDownLim - RotateBufferUpDown) {
            // allow up but not down
            slideRotate.setPower(Math.min(speed, 0));
            slideRotate2.setPower(Math.min(speed, 0));
        } else if (slideRotate.getCurrentPosition() < RotateUpLim + RotateBufferUpDown) {
            // allow down but not up
            slideRotate.setPower(Math.max(speed, 0));
            slideRotate2.setPower(Math.max(speed, 0));
        } else {
            // allow both up and down
            slideRotate.setPower(speed);
            slideRotate2.setPower(speed);
        }

    }

    private double GetRemainingSlideDistance(int SlideRotate_Encoder, int Slide_Encoder) {
        double ret;

        // This function gets the remaining encoder value allowed for the slide to extend for the given slideRotate encoder position.
        // This includes limits on max slide encoding allowed.
        // This will be negative if the slide is past the allowed distance.
        double theta = getAngle(SlideRotate_Encoder);
        double v = (SlideArmLength + (SlideMaxLength / SlideMaxEncoder) * Slide_Encoder) * Math.cos(theta / 180 * Math.PI);
        if (theta < 90) {
            // arm in front of pivot
            ret = ((SlideAllowedInSlideCanGoPastFrontOfRobot - (SlidePositionInRobotFromBack + SlideAllowedOffsetInSlideCanGoPastBackOfRobot)) - v) / Math.cos(theta / 180 * Math.PI);
        } else if (theta > 90) {
            // arm in behind pivot, ie. angle >90
            ret = (SlidePositionInRobotFromBack + SlideAllowedOffsetInSlideCanGoPastBackOfRobot + v) / -Math.cos(theta / 180 * Math.PI);
        } else {
            // arm is vertical, so set ret to a very large number. Fixes divide by 0 error giving negative number at cos(90) = 0.
            // this number is a little under 2^31.
            ret = Math.pow(10, 9);
        }
        return Math.min(ret * (SlideMaxEncoder / SlideMaxLength), SlideMaxEncoder - Slide_Encoder);
    }
    public double GetRemainingSlideDistance() {
        return GetRemainingSlideDistance((slideRotate.getCurrentPosition() + slideRotate2.getCurrentPosition())/2, (slide.getCurrentPosition()+slide2.getCurrentPosition())/2);
    }

    /*public void MoveSlideTMP(String preset, Telemetry telemetry) {

        Presets preset1;
        if (preset.equals("WallPickup")) {
            preset1 = Presets.WallPickup;
        } else if (preset.equals("Ascent")) {
            preset1 = Presets.Ascent;
        } else if (preset.equals("TopSpecimen")) {
            preset1 = Presets.TopSpecimen;
        } else if (preset.equals("DropTopSpecimen")) {
            preset1 = Presets.DropTopSpecimen;
        } else {
            throw new Error("PRESET NOT VALID: " + preset);
        }
        telemetry.addLine("STARTING TELEMETRY MOVESLIDETMP");
        telemetry.update();
        sleep(2000);
        slide.setTargetPosition(preset1.slideEncoder);
        slide2.setTargetPosition(preset1.slideEncoder);
        telemetry.addLine("Started target position slide");
        telemetry.update();
        sleep(2000);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("Started run to position slide");
        telemetry.update();
        sleep(2000);
        slide.setPower(0.7);
        slide2.setPower(0.7);
        telemetry.addLine("Set power slide");
        telemetry.update();
        sleep(2000);

        telemetry.addLine("DONE!!!");
        telemetry.update();
        sleep(2000);

        stateSlide = State.Preset;
    }*/

    public void MoveSlide(Presets preset, double speed) {
        // check if encoders don't match, with buffer
        if (Math.abs(slide.getCurrentPosition() - slide2.getCurrentPosition()) > SlideBuffer) {
            stateSlide = State.Broken;
        } else if (stateSlide == State.Broken) {
            // state is broken, but within limits
            stateSlide = State.DriverControlled;
            if (VModeSlide) {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        // stop slide motors if broken
        if (stateSlide == State.Broken) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(0);
            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide2.setPower(0);
            return;
        }/*
        Presets preset1;
        if (preset.equals("WallPickup")) {
            preset1 = Presets.WallPickup;
        } else if (preset.equals("Ascent")) {
            preset1 = Presets.Ascent;
        } else if (preset.equals("TopSpecimen")) {
            preset1 = Presets.TopSpecimen;
        } else if (preset.equals("DropTopSpecimen")) {
            preset1 = Presets.DropTopSpecimen;
        } else {
            throw new Error("PRESET NOT VALID: " + preset);
        }*/

        double remSlideDist = GetRemainingSlideDistance();
        int slideEncoder = (slide.getCurrentPosition() + slide2.getCurrentPosition())/2;

        if (remSlideDist < 0) {
            // past allowed position, cancel move to position and retract
            if (VModeSlide) {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            slide.setPower(-1);
            slide2.setPower(-1);
            stateSlide = State.ForcedRetract;
            return;
        } else if (remSlideDist < SlideBufferUp && preset.slideEncoder > slideEncoder) {
            // within buffer and moving up more, cancel movement.
            if (VModeSlide) {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            slide.setPower(0);
            slide2.setPower(0);
            stateSlide = State.ForcedRetract;
            return;
        } else {
            // go to position number
            // TODO do I need a check for if already at position?
            slide.setTargetPosition(preset.slideEncoder);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(speed);
            slide2.setTargetPosition(preset.slideEncoder);
            slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide2.setPower(speed);
            stateSlide = State.Preset;
        }
    }
    public void MoveSlide(Presets preset) {
        MoveSlide(preset, 0.8);
    }
    public void MoveSlide(double speed) {
        // check if encoders don't match, with buffer
        if (Math.abs(slide.getCurrentPosition() - slide2.getCurrentPosition()) > SlideBuffer) {
            stateSlide = State.Broken;
        } else if (stateSlide == State.Broken) {
            // state is broken, but within limits
            stateSlide = State.Preset;
        }

        // stop slide motors if broken
        if (stateSlide == State.Broken) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(0);
            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide2.setPower(0);
            return;
        }
        double remSlideDist = GetRemainingSlideDistance();
        //int slideEncoder = (slide.getCurrentPosition() + slide2.getCurrentPosition())/2;

        if (stateSlide != State.DriverControlled) {
            if (VModeSlide) {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (remSlideDist < 0) {
            // force retract slide because it is past the limit
            slide.setPower(-1);
            slide2.setPower(-1);
            stateSlide = State.ForcedRetract;
            return;
        } else if (remSlideDist < SlideBufferUp) {
            // allow retraction but not expansion
            slide.setPower(Math.min(speed, 0));
            slide2.setPower(Math.min(speed, 0));
        } else if (remSlideDist > SlideMaxEncoder - SlideBufferDown) {
            // allow expansion but not retraction
            slide.setPower(Math.max(speed, 0));
            slide2.setPower(Math.max(speed, 0));
        } else {
            // allow both retraction and expansion
            slide.setPower(speed);
            slide2.setPower(speed);
        }
    }

    public void Sync() {
        slide.setTargetPosition(slide2.getCurrentPosition());
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.4);
        slide2.setTargetPosition(slide2.getCurrentPosition());
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setPower(0.4);
        slideRotate.setTargetPosition(slideRotate2.getCurrentPosition());
        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotate.setPower(0.3);
        slideRotate2.setTargetPosition(slideRotate2.getCurrentPosition());
        slideRotate2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotate2.setPower(0.3);
        sleep(1000);
    }
}
