package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Action {
    // place to go
    Pose2D goTo;

    // slide preset to go to
    SlidesAndRotate.Presets slidePreset;

    Action(Pose2D goTo, SlidesAndRotate.Presets slidePreset) {
        this.goTo = goTo;
        this.slidePreset = slidePreset;
    }
}
