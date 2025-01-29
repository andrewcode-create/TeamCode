package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// All constants in Pose2D.
// X is forward/backward (positive is forward)
// Y is left/right (positive is left)
// Angles are normalized, with positive being to the left (counterclockwise from above)
// (0,0) is considered to be the corner of the observation zone.
public class Constants {
    public static final double pinpointRotX = 100;
    public static final double pinpointRotY = 100;


    // starting position of the robot
    public static final Pose2D startingPosAuto = new Pose2D(DistanceUnit.MM, 0, 1000, AngleUnit.DEGREES, 0);
    public static final Pose2D startingPosTeleop = new Pose2D(DistanceUnit.MM, 0, 1000, AngleUnit.DEGREES, 0);

    // specimen pickup from observation zone
    public static final Pose2D pickUpSpecimen = new Pose2D(DistanceUnit.MM, 50, 200, AngleUnit.DEGREES, 180);

    // Autonomous points for high bar
    public static final Pose2D Drop1 = new Pose2D(DistanceUnit.MM, 800, 1300, AngleUnit.DEGREES, 0);
    public static final Pose2D Drop2 = new Pose2D(DistanceUnit.MM, 800, 1400, AngleUnit.DEGREES, 0);
    public static final Pose2D Drop3 = new Pose2D(DistanceUnit.MM, 800, 1500, AngleUnit.DEGREES, 0);
    public static final Pose2D Drop4 = new Pose2D(DistanceUnit.MM, 800, 1600, AngleUnit.DEGREES, 0);
    public static final Pose2D Drop5 = new Pose2D(DistanceUnit.MM, 800, 1700, AngleUnit.DEGREES, 0);


    // Autonomous point for go-around between bar and ground
    public static final Pose2D goBarGround = new Pose2D(DistanceUnit.MM, 400, 800, AngleUnit.DEGREES, 0);

    // Autonomous points to go to for pushing ground specimen into observation zone, and go-around points
    public static final Pose2D ground1 = new Pose2D(DistanceUnit.MM, 1600, 1000, AngleUnit.DEGREES, 180);
    public static final Pose2D goGround1 = new Pose2D(DistanceUnit.MM, 1600, 1200, AngleUnit.DEGREES, 180);
    public static final Pose2D ground2 = new Pose2D(DistanceUnit.MM, 1600, 800, AngleUnit.DEGREES, 180);
    public static final Pose2D goGround2 = new Pose2D(DistanceUnit.MM, 1600, 1000, AngleUnit.DEGREES, 180);
    public static final Pose2D ground3 = new Pose2D(DistanceUnit.MM, 1600, 0, AngleUnit.DEGREES, 180);
    public static final Pose2D goGround3 = new Pose2D(DistanceUnit.MM, 1600, 300, AngleUnit.DEGREES, 180);
}
