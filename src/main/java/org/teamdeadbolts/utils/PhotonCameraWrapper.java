/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraWrapper {
    private final Transform3d offset;
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private Pose3d averagedTargetToRobotPose = new Pose3d();
    private int avgCount = 0;
    private boolean hasTargets = false;
    private int bestTagId = -1;
    private double timestamp = 0;

    /**
     * @param camName The name of the PhotonCamera.
     * @param offset The Transform3d from the Robot's center to the Camera's lens.
     * @param fieldLayout The loaded AprilTagFieldLayout for the current competition.
     */
    public PhotonCameraWrapper(String camName, Transform3d offset) {
        this.camera = new PhotonCamera(camName);
        this.offset = offset;
    }

    public void update() {
        Logger.recordOutput("Vision/Camera " + camera.getName() + "/HasTargets", this.hasTargets);
        Logger.recordOutput("Vision/Camera " + camera.getName() + "/AvgCount", this.avgCount);
        Logger.recordOutput("Vision/Camera " + camera.getName() + "/BestTagId", this.bestTagId);
        this.getAvgFieldRelativePose()
                .ifPresent(
                        pose -> {
                            Logger.recordOutput(
                                    "Vision/Camera " + camera.getName() + "/EstimatedPose", pose);
                        });

        Translation3d currTranslationSum = new Translation3d();
        Rotation3d firstRotation = null;
        int validResCount = 0;

        this.bestTagId = -1;
        this.hasTargets = false;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            this.avgCount = 0;
            return;
        }

        PhotonPipelineResult bestResult =
                results.stream().filter(PhotonPipelineResult::hasTargets).findFirst().orElse(null);

        if (bestResult == null) {
            this.avgCount = 0;
            return;
        }

        this.hasTargets = true;
        this.timestamp = bestResult.getTimestampSeconds();
        this.bestTagId = bestResult.getBestTarget().getFiducialId();

        for (PhotonPipelineResult r : results) {
            if (r.hasTargets()) {
                PhotonTrackedTarget best = r.getBestTarget();

                Transform3d camToTarget = best.getBestCameraToTarget();

                Transform3d targetToCam = camToTarget.inverse();

                Transform3d targetToRobot = targetToCam.plus(this.offset.inverse());

                currTranslationSum = currTranslationSum.plus(targetToRobot.getTranslation());

                if (firstRotation == null) {
                    firstRotation = targetToRobot.getRotation();
                }
                validResCount++;
            }
        }

        if (validResCount > 0) {
            Translation3d avgTrans = currTranslationSum.div(validResCount);

            this.averagedTargetToRobotPose =
                    new Pose3d(avgTrans, firstRotation != null ? firstRotation : new Rotation3d());
            this.avgCount = validResCount;
        } else {
            this.avgCount = 0;
        }
    }

    /**
     * Calculates the field-relative robot pose using the averaged target-relative pose
     * and the known field pose of the detected AprilTag.
     * * Formula: Field Pose = Tag Field Pose + Target Relative Robot Pose
     * * @return An Optional containing the Field-Relative Pose3d, or empty if no targets are seen
     * OR the tag ID is not found in the Field Layout.
     */
    public Optional<Pose3d> getAvgFieldRelativePose() {
        if (!this.hasTargets || this.bestTagId == -1 || this.avgCount == 0) {
            return Optional.empty();
        }

        Optional<Pose3d> tagFieldPose = fieldLayout.getTagPose(this.bestTagId);

        if (tagFieldPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d finalFieldRelativePose =
                tagFieldPose
                        .get()
                        .plus(
                                new Transform3d(
                                        averagedTargetToRobotPose.getTranslation(),
                                        averagedTargetToRobotPose.getRotation()));

        return Optional.of(finalFieldRelativePose);
    }

    /**
     * @return The number of results included in the latest average calculation.
     */
    public int getAvgCount() {
        return this.avgCount;
    }

    /**
     * @return True if targets were detected in the last update cycle.
     */
    public boolean hasTargets() {
        return this.hasTargets;
    }

    // Get the timestamp of the latest pose
    public double getTimestamp() {
        return this.timestamp;
    }
}
