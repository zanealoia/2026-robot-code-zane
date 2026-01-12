/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.state.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOCtx {
        public PoseObservation[] observations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    public static record PoseObservation(
            double timestamp, Pose3d pose, double ambiguity, double tagDist) {}

    public default void update(VisionIOCtx ctx) {}
}
