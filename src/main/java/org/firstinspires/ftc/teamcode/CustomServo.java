package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class CustomServo {
    private Servo servo;

    private final double openPos;
    private final double closePos;
    private final double changePos; // change per second
    private final double midPos;
    private boolean isDisabled = false;
    private Position pos;
    public Position getPosition() {return pos;}

    public CustomServo(double open, double close, double mid, double change) {
        openPos = open;
        closePos = close;
        midPos = mid;
        changePos = change;
    }
    public CustomServo(double open, double close) {
        this(open, close, (open+close)/2, 0.5);
    }
    public CustomServo(double open, double close, double mid) {
        this(open, close, mid, 0.5);
    }

    public enum Position {
        open,
        close,
        mid,
        none,
    }

    public void init(Servo servo, Position startingPosition) {
        this.servo = servo;
        // increase range of servo to maximum. Rev smart servo and goBuilda servos support this.
        ((ServoImplEx)servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        moveToPos(startingPosition);
    }

    // disables the servo until next move command. On rev and goBuilda servos, this de-energises.
    public void disableServo() {
        servo.getController().pwmDisable();
        isDisabled = true;
        pos = Position.none;
    }

    public void moveToPos(Position newPos) {
        if (isDisabled) {
            servo.getController().pwmEnable();
            isDisabled = false;
        }
        if (newPos == Position.open) {
            servo.setPosition(openPos);
            pos = Position.open;
        } else if (newPos == Position.close) {
            servo.setPosition(closePos);
            pos = Position.close;
        } else if (newPos == Position.mid) {
            servo.setPosition(midPos);
            pos = Position.mid;
        }
    }

    public void move(long msElapsed, boolean forwards) {
        if (isDisabled) {
            servo.getController().pwmEnable();
            isDisabled = false;
        }
        if (forwards) {
            servo.setPosition(servo.getPosition() + msElapsed*changePos/1000);
        } else {
            servo.setPosition(servo.getPosition() - msElapsed*changePos/1000);
        }
        pos = Position.none;
    }

}
