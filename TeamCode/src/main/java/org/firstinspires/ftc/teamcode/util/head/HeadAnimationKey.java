package org.firstinspires.ftc.teamcode.util.head;

public class HeadAnimationKey {
    public enum TransitionType {
        LINEAR,
        EASE_IN,
        EASE_OUT,
        EASE_IN_OUT
    }

    private final HeadPosition position;
    private final int time;
    private final TransitionType transitionType;

    public HeadAnimationKey(HeadPosition position, int time, TransitionType transitionType) {
        this.position = position;
        this.time = time;
        this.transitionType = transitionType;
    }
}
