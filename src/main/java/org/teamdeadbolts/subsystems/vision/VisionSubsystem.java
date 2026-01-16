/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.constants.VisionConstants;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.vision.PhotonVisionIO.PhotonVisionIOCtx;
import org.teamdeadbolts.subsystems.vision.PhotonVisionIO.PoseObservation;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkBoolean;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class VisionSubsystem extends SubsystemBase {
    private Pose3d estimatedPose = new Pose3d();

    private RobotState robotState = RobotState.getInstance();
    // private SwerveSubsystem swerveSubsystem;

    // private SwerveDrivePoseEstimator3d poseEstimator3d;

    private PhotonVisionIOCtx[] ctxs;
    private PhotonVisionIO[] ios;

    // TODO
    SavedLoggedNetworkNumber maxAmbiguity = SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/MaxAmbiguity", 0.5);

    SavedLoggedNetworkNumber maxTagDist = SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/MaxTagDist", 0.5);

    SavedLoggedNetworkBoolean enableVision = SavedLoggedNetworkBoolean.get("Tuning/PoseEstimator/EnableVision", true);

    private final ArrayList<Pose3d> tagPoses = new ArrayList<>();
    private final ArrayList<Pose3d> robotPoses = new ArrayList<>();
    private int loopCount = 0;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem, PhotonVisionIO... ios) {
        this.ios = ios;
        this.ctxs = new PhotonVisionIOCtx[ios.length];
        for (int i = 0; i < ios.length; i++) {
            this.ctxs[i] = new PhotonVisionIOCtx();
        }
        // this.swerveSubsystem = swerveSubsystem;
        // swerveSubsystem.setModulePositionCallback(this::updateFromSwerve);
    }

    // private void updateFromSwerve(SwerveModulePosition[] positions, Rotation2d gyroRotation) {
    //     estimatedPose = poseEstimator3d.update(new Rotation3d(gyroRotation), positions);
    // }

    // public void setPosition(Pose3d pose) {
    //     this.poseEstimator3d.resetPosition(
    //             new Rotation3d(this.swerveSubsystem.getGyroRotation()), swerveSubsystem.getModulePositions(), pose);
    // }


    @Override
    public void periodic() {
        long startTime = RobotController.getFPGATime();
        Tracer tracer = new Tracer();

        tracer.addEpoch("Start");
        Logger.recordOutput("PoseEstimator/Pose3d", estimatedPose);
        robotState.setEstimatedPose(estimatedPose);

        tracer.addEpoch("Set Robot State");

        tagPoses.clear();
        robotPoses.clear();
        for (int i = 0; i < ios.length; i++) {
            ios[i].update(ctxs[i]);
            // Logger.processInputs("Vision/Camera " + i, ctxs[i]);
        }

        tracer.addEpoch("IO + Inputs");

        for (int index = 0; index < ctxs.length; index++) {
            for (int tId : ctxs[index].tagIds) {
                Optional<Pose3d> tagPose = VisionConstants.FIELD_LAYOUT.getTagPose(tId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            tracer.addEpoch("Tag Lookup");

            for (PoseObservation observation : ctxs[index].observations) {
                boolean acceptPose = observation.ambiguity() <= maxAmbiguity.get()
                        && observation.tagDist() <= maxTagDist.get()
                        && observation.pose().getX() > 0.0
                        && observation.pose().getX() < VisionConstants.FIELD_LAYOUT.getFieldLength()
                        && observation.pose().getY() > 0.0
                        && observation.pose().getY() < VisionConstants.FIELD_LAYOUT.getFieldWidth();
                if (acceptPose) {
                    robotPoses.add(observation.pose());
                }

                if (enableVision.get()) {
                    if (!acceptPose) continue;
                    double stdDevFactor = observation.tagDist() * Math.sqrt(observation.tagDist());
                    // double transStdDev = visionTransStdDev.get() * stdDevFactor;
                    // double headingStdDev = visionHeadingStdDev.get() * stdDevFactor;

                    // poseObservationConsumer.accept(observation
                    RobotState.getInstance()
                            .addVisionMeasurement(
                                    observation.pose(),
                                    observation.timestamp(),
                                    observation.tagDist());
                }
            }

            tracer.addEpoch("Observations");

            // Logging
            if (loopCount++ > 4) {
                loopCount = 0;

                Logger.recordOutput(
                        "Vision/Camera " + index + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
                Logger.recordOutput(
                        "Vision/Camera " + index + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
            }

            tracer.addEpoch("Logging");
        }

        long elapsed = RobotController.getFPGATime() - startTime;
        if (elapsed > 1_000_000) {
            tracer.printEpochs();
        }
    }
}