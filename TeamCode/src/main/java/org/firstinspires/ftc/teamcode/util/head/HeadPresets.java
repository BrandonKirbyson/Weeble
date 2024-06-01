package org.firstinspires.ftc.teamcode.util.head;

import java.util.ArrayList;

public class HeadPresets {
    public static final ArrayList<HeadAnimationKey> NodYes = new ArrayList<HeadAnimationKey>() {{
        add(new HeadAnimationKey(new HeadPosition(0.5, 0.5), 1000, HeadAnimationKey.TransitionType.EASE_IN_OUT));
        add(new HeadAnimationKey(new HeadPosition(0.5, 0.5, 0.7), 100, HeadAnimationKey.TransitionType.EASE_IN_OUT));
    }};

    public static final ArrayList<HeadAnimationKey> ShakeNo = new ArrayList<HeadAnimationKey>() {{
        for (int i = 0; i < 3; i++) {
            add(new HeadAnimationKey(new HeadPosition(0.5, 0.5), 1000, HeadAnimationKey.TransitionType.EASE_IN_OUT));
            add(new HeadAnimationKey(new HeadPosition(0.5, 0.5, 0.3), 100, HeadAnimationKey.TransitionType.EASE_IN_OUT));
        }
    }};
}
