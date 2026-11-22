package org.firstinspires.ftc.teamcode.auto.steps;

import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.subsys.Vision;

/**
 * The one place that turns the Limelight's AprilTag count into a hive judgement, so every branch of every route
 * decides the same way.
 */
public final class HiveView {
    private final Vision vision;

    public HiveView(Vision vision) {
        this.vision = vision;
    }

    /**
     * @return true if the Limelight currently sees MORE than {@link Tunables#tagThreshold} tags. From the audience
     * side that means the hive is NOT tipped toward the scoring side; no tags means it is.
     */
    public boolean tagsVisible() {
        return vision.getLatestResult().getBotposeTagCount() > Tunables.tagThreshold;
    }
}
