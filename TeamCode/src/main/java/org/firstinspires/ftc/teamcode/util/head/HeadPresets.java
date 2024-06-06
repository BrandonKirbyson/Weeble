package org.firstinspires.ftc.teamcode.util.head;

import java.util.ArrayList;

public class HeadPresets {
    public static final ArrayList<HeadAnimationKeyframe> NodYes = new ArrayList<HeadAnimationKeyframe>() {{
        add(new HeadAnimationKeyframe(new HeadOrientation(0.5, 0.5), 1000, HeadAnimationKeyframe.TransitionType.EASE_IN_OUT));
        add(new HeadAnimationKeyframe(new HeadOrientation(0.5, 0.5, 0.7), 100, HeadAnimationKeyframe.TransitionType.EASE_IN_OUT));
    }};

    public static final ArrayList<HeadAnimationKeyframe> ShakeNo = new ArrayList<HeadAnimationKeyframe>() {{
        for (int i = 0; i < 3; i++) {
            add(new HeadAnimationKeyframe(new HeadOrientation(0.5, 0.5), 1000, HeadAnimationKeyframe.TransitionType.EASE_IN_OUT));
            add(new HeadAnimationKeyframe(new HeadOrientation(0.5, 0.5, 0.3), 100, HeadAnimationKeyframe.TransitionType.EASE_IN_OUT));
        }
    }};
}
