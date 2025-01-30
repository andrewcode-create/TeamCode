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
    public static final Pose2D startingPosAuto = new Pose2D(DistanceUnit.MM, 0, 1355, AngleUnit.DEGREES, 0);
    public static final Pose2D startingPosTeleop = new Pose2D(DistanceUnit.MM, 0, 1330, AngleUnit.DEGREES, 0);

    // specimen pickup from observation zone
    public static final Pose2D pickUpSpecimen = new Pose2D(DistanceUnit.MM, 165, 750, AngleUnit.DEGREES, 180);

    // Autonomous points for high bar
    public static final Pose2D Drop1 = new Pose2D(DistanceUnit.MM, 810, 1470, AngleUnit.DEGREES, 0);
    public static final Pose2D Drop2 = new Pose2D(DistanceUnit.MM, 810, 1510, AngleUnit.DEGREES, 0);
    public static final Pose2D Drop3 = new Pose2D(DistanceUnit.MM, 810, 1560, AngleUnit.DEGREES, 0);
    public static final Pose2D Drop4 = new Pose2D(DistanceUnit.MM, 810, 1600, AngleUnit.DEGREES, 0);
    public static final Pose2D Drop5 = new Pose2D(DistanceUnit.MM, 810, 1650, AngleUnit.DEGREES, 0);


    // Autonomous point for go-around between bar and ground
    public static final Pose2D goBarGround = new Pose2D(DistanceUnit.MM, 670, 1450, AngleUnit.DEGREES, 0);

    public static final Pose2D goBarGround2 = new Pose2D(DistanceUnit.MM, 660, 660, AngleUnit.DEGREES, 0);

    // Autonomous points to go to for pushing ground specimen into observation zone, and go-around points
    public static final Pose2D ground1 = new Pose2D(DistanceUnit.MM, 1600, 1000, AngleUnit.DEGREES, 180);
    public static final Pose2D goGround1 = new Pose2D(DistanceUnit.MM, 1600, 1200, AngleUnit.DEGREES, 180);
    public static final Pose2D ground2 = new Pose2D(DistanceUnit.MM, 1150, 610, AngleUnit.DEGREES, 180);
    public static final Pose2D goGround2 = new Pose2D(DistanceUnit.MM, 840, 530, AngleUnit.DEGREES, -90);

    // go to observation zone for 3rd ground
    public static final Pose2D ground3 = new Pose2D(DistanceUnit.MM, -200, 290, AngleUnit.DEGREES, -90);

    // pickup 3rd ground
    public static final Pose2D goGround3 = new Pose2D(DistanceUnit.MM, 830, 280, AngleUnit.DEGREES, -90);



    /*
    Plan for auto:
    1. put on high bar, position 5
    2. go back 160 mm, continue
    3. go to x570, y930, r-90, stop
    4. go to x810, y910, r-90, stop
    5. go to x810, y752, r-90, stop
    6. go to drop off ( x170 y760, r180 )
    7. go to x810, y250, r-90, stop
    8. go to x400, y260, r-90, continue
    9. go to x150, y466, r180, stop & pick up
    10. go to x400, y466, r180, continue
    11. go to 160mm behind high bar, continue
    12. put on high bar
    13. go back 160 mm, continue
    14. go to pick up (x170, y760, r180)
    15. repeat 11-15 4 times
    16. park at drop off
     */
}
