package org.firstinspires.ftc.teamcode.util.head;

public class HeadAnimationKeyframe {
    public enum TransitionType {
        LINEAR,
        EASE_IN,
        EASE_OUT,
        EASE_IN_OUT
    }

    public final HeadOrientation position;
    public final int time;
    public final TransitionType transitionType;

    public HeadAnimationKeyframe(HeadOrientation position, int time, TransitionType transitionType) {
        this.position = position;
        this.time = time;
        this.transitionType = transitionType;
    }

    private static double Bezier(double progress) {
        return (progress * progress) * (3 - (2 * progress));
    }

    public static double getTransitionProgress(double progress, TransitionType transitionType) {
        switch (transitionType) {
            case EASE_IN:
                if (progress <= 0.5) {
                    return Bezier(progress);
                } else {
                    return progress;
                }
            case EASE_OUT:
                if (progress >= 0.5) {
                    return Bezier(progress);
                } else {
                    return progress;
                }
            case EASE_IN_OUT:
                return Bezier(progress);
        }
        return progress;
    }
}
