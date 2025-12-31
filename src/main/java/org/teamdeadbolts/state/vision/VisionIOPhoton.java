/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.state.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.teamdeadbolts.constants.VisionConstants;

public class VisionIOPhoton implements VisionIO {
    private final Transform3d offset;
    private final PhotonCamera camera;

    /**
     * @param camName The name of the PhotonCamera.
     * @param offset The Transform3d from the Robot's center to the Camera's lens.
     * @param fieldLayout The loaded AprilTagFieldLayout for the current competition.
     */
    public VisionIOPhoton(String camName, Transform3d offset) {
        this.camera = new PhotonCamera(camName);
        this.offset = offset;
    }

    public void update(VisionIOCtx ctx) {

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return;
        }

        HashSet<Integer> tagIds = new HashSet<>();
        LinkedList<PoseObservation> poseObservations = new LinkedList<>();

        for (PhotonPipelineResult r : results) {

            if (r.hasTargets()) {
                PhotonTrackedTarget best = r.getBestTarget();
                Optional<Pose3d> tagPose = VisionConstants.FIELD_LAYOUT.getTagPose(best.fiducialId);

                if (tagPose.isPresent()) {
                    // System.out.println("Got tag " + best.fiducialId);
                    Transform3d fieldToTarget =
                            new Transform3d(
                                    tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d camToTarget = best.bestCameraToTarget;
                    Transform3d fieldToCam = fieldToTarget.plus(camToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCam.plus(offset.inverse());
                    Pose3d robotPose =
                            new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
                    tagIds.add(best.fiducialId);
                    poseObservations.add(
                            new PoseObservation(
                                    r.getTimestampSeconds(),
                                    robotPose,
                                    best.poseAmbiguity,
                                    camToTarget.getTranslation().getNorm()));
                }
            }
        }

        ctx.observations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            ctx.observations[i] = poseObservations.get(i);
        }

        ctx.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int tagId : tagIds) {
            ctx.tagIds[i++] = tagId;
        }
    }
}
