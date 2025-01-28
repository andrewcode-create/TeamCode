package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public enum Action {
    wallPickup(new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.RADIANS, Math.PI), SlidesAndRotate.Presets.WallPickup),
    slidePlace(new Pose2D(DistanceUnit.MM,800, 0, AngleUnit.RADIANS, 0), SlidesAndRotate.Presets.TopSpecimen),
    pointToGet(new Pose2D(DistanceUnit.MM,1000, 600, AngleUnit.RADIANS, Math.PI), SlidesAndRotate.Presets.WallPickup),
    get1(new Pose2D(DistanceUnit.MM,1000, 300, AngleUnit.RADIANS, Math.PI), SlidesAndRotate.Presets.WallPickup),
    get2(new Pose2D(DistanceUnit.MM,1000, 150, AngleUnit.RADIANS, Math.PI), SlidesAndRotate.Presets.WallPickup);

    // place to go
    final Pose2D goTo;

    // slide preset to go to
    final SlidesAndRotate.Presets slidePreset;

    Action(Pose2D goTo, SlidesAndRotate.Presets slidePreset) {
        this.goTo = goTo;
        goTo.getHeading(AngleUnit.RADIANS);
        this.slidePreset = slidePreset;
    }

    public boolean isDone(Pose2D currentPos, Pose2D errorTolerance) {
        return  Math.abs(currentPos.getHeading(AngleUnit.DEGREES) - goTo.getHeading(AngleUnit.DEGREES)) <= errorTolerance.getHeading(AngleUnit.DEGREES) &&
                Math.abs(currentPos.getX(DistanceUnit.MM) - goTo.getX(DistanceUnit.MM)) <= errorTolerance.getX(DistanceUnit.MM) &&
                Math.abs(currentPos.getY(DistanceUnit.MM) - goTo.getY(DistanceUnit.MM)) <= errorTolerance.getY(DistanceUnit.MM);
    }
}
