package org.firstinspires.ftc.teamcode.util.head;

public class HeadAnimationKeyframe {
    public enum TransitionType {
        LINEAR,
        EASE_IN,
        EASE_OUT,
        EASE_IN_OUT
    }

    public final HeadPosition position;
    public final int time;
    public final TransitionType transitionType;

    public HeadAnimationKeyframe(HeadPosition position, int time, TransitionType transitionType) {
        this.position = position;
        this.time = time;
        this.transitionType = transitionType;
    }
}
