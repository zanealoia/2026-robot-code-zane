/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.state;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.LinkedList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.constants.VisionConstants;
import org.teamdeadbolts.state.vision.VisionIO;
import org.teamdeadbolts.state.vision.VisionIO.PoseObservation;
import org.teamdeadbolts.state.vision.VisionIOCtxAutoLogged;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkBoolean;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class PoseEstimator {
    private Pose3d estimatedPose = new Pose3d();

    private RobotState robotState = RobotState.getInstance();
    private SwerveSubsystem swerveSubsystem;

    private SwerveDrivePoseEstimator3d poseEstimator3d;

    private VisionIOCtxAutoLogged[] ctxs;
    private VisionIO[] ios;

    /* Tuning values */
    SavedLoggedNetworkNumber wheelTransStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/WheelTransStdDev", 0.05);
    SavedLoggedNetworkNumber wheelHeadingStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/WheelHeadingStdDev", 0.05);

    /* Vision base std dev (sacles with distance) */
    SavedLoggedNetworkNumber visionTransStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/VisionTransStdDev", 0.05);
    SavedLoggedNetworkNumber visionHeadingStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/VisionHeadingStdDev", 0.05);

    // TODO
    SavedLoggedNetworkNumber maxAmbiguity =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/MaxAmbiguity", 0.5);

    SavedLoggedNetworkNumber maxTagDist =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/MaxTagDist", 0.5);

    SavedLoggedNetworkBoolean enableVision =
            SavedLoggedNetworkBoolean.get("Tuning/PoseEstimator/EnableVision", true);

    public PoseEstimator(SwerveSubsystem swerveSubsystem, VisionIO... ios) {
        this.ios = ios;
        this.ctxs = new VisionIOCtxAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            this.ctxs[i] = new VisionIOCtxAutoLogged();
        }
        this.poseEstimator3d =
                new SwerveDrivePoseEstimator3d(
                        SwerveConstants.SWERVE_KINEMATICS,
                        new Rotation3d(swerveSubsystem.getGyroRotation()),
                        swerveSubsystem.getModulePositions(),
                        new Pose3d(),
                        VecBuilder.fill(
                                wheelTransStdDev.get(),
                                wheelTransStdDev.get(),
                                wheelTransStdDev.get(),
                                wheelHeadingStdDev.get()),
                        VecBuilder.fill(
                                visionTransStdDev.get(),
                                visionTransStdDev.get(),
                                visionTransStdDev.get(),
                                visionHeadingStdDev.get()));
        this.swerveSubsystem = swerveSubsystem;
        swerveSubsystem.setModulePositionCallback(this::updateFromSwerve);
    }

    private void updateFromSwerve(SwerveModulePosition[] positions, Rotation2d gyroRotation) {
        estimatedPose = poseEstimator3d.update(new Rotation3d(gyroRotation), positions);
    }

    public void setPosition(Pose3d pose) {
        this.poseEstimator3d.resetPosition(
                new Rotation3d(this.swerveSubsystem.getGyroRotation()),
                swerveSubsystem.getModulePositions(),
                pose);
    }

    public void update() {
        Logger.recordOutput("PoseEstimator/Pose3d", estimatedPose);
        robotState.setRobotPose(estimatedPose);

        for (int i = 0; i < ios.length; i++) {
            ios[i].update(ctxs[i]);
            Logger.processInputs("Vision/Camera " + i, ctxs[i]);
        }

        for (int index = 0; index < ctxs.length; index++) {
            LinkedList<Pose3d> tagPoses = new LinkedList<>();
            LinkedList<Pose3d> robotPoses = new LinkedList<>();

            for (int tId : ctxs[index].tagIds) {
                Optional<Pose3d> tagPose = VisionConstants.FIELD_LAYOUT.getTagPose(tId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            for (PoseObservation observation : ctxs[index].observations) {
                boolean acceptPose =
                        observation.ambiguity() <= maxAmbiguity.get()
                                && observation.tagDist() <= maxTagDist.get()
                                && observation.pose().getX() > 0.0
                                && observation.pose().getX()
                                        < VisionConstants.FIELD_LAYOUT.getFieldLength()
                                && observation.pose().getY() > 0.0
                                && observation.pose().getY()
                                        < VisionConstants.FIELD_LAYOUT.getFieldWidth();
                if (acceptPose) {
                    robotPoses.add(observation.pose());
                }

                if (enableVision.get()) {
                    if (!acceptPose) continue;
                    double stdDevFactor = Math.pow(observation.tagDist(), 1.5);
                    double transStdDev = visionTransStdDev.get() * stdDevFactor;
                    double headingStdDev = visionHeadingStdDev.get() * stdDevFactor;

                    poseEstimator3d.addVisionMeasurement(
                            observation.pose(),
                            observation.timestamp(),
                            VecBuilder.fill(transStdDev, transStdDev, transStdDev, headingStdDev));
                }
            }

            // Logging
            Logger.recordOutput(
                    "Vision/Camera " + index + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera " + index + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
        }
    }
}
