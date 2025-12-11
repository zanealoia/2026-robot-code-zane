/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.state;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class PoseEstimator {
    private Pose3d estimatedPose = new Pose3d();

    private RobotState robotState = RobotState.getInstance();
    private SwerveSubsystem swerveSubsystem;

    private SwerveDrivePoseEstimator3d poseEstimator3d;

    /* Tuning values */
    SavedLoggedNetworkNumber positionStdDevX =
            new SavedLoggedNetworkNumber("Tuning/PoseEstimator/PositionStdDevX", 0.05);
    SavedLoggedNetworkNumber positionStdDevY =
            new SavedLoggedNetworkNumber("Tuning/PoseEstimator/PositionStdDevY", 0.05);
    SavedLoggedNetworkNumber positionStdDevZ =
            new SavedLoggedNetworkNumber("Tuning/PoseEstimator/PositionStdDevZ", 0.05);
    SavedLoggedNetworkNumber headingStdDev =
            new SavedLoggedNetworkNumber("Tuning/PoseEstimator/HeadingStdDev", 0.05);

    /* Vision base std dev (sacles with distance) */
    SavedLoggedNetworkNumber visionStdDevX =
            new SavedLoggedNetworkNumber("Tuning/PoseEstimator/VisionStdDevX", 0.05);
    SavedLoggedNetworkNumber visionStdDevY =
            new SavedLoggedNetworkNumber("Tuning/PoseEstimator/VisionStdDevY", 0.05);
    SavedLoggedNetworkNumber visionStdDevZ =
            new SavedLoggedNetworkNumber("Tuning/PoseEstimator/VisionStdDevZ", 0.05);
    SavedLoggedNetworkNumber visionHeadingStdDev =
            new SavedLoggedNetworkNumber("Tuning/PoseEstimator/VisionHeadingStdDev", 0.05);

    public PoseEstimator(SwerveSubsystem swerveSubsystem) {
        this.poseEstimator3d =
                new SwerveDrivePoseEstimator3d(
                        SwerveConstants.SWERVE_KINEMATICS,
                        new Rotation3d(swerveSubsystem.getGyroRotation()),
                        swerveSubsystem.getModulePositions(),
                        new Pose3d(),
                        VecBuilder.fill(
                                positionStdDevX.get(),
                                positionStdDevY.get(),
                                positionStdDevZ.get(),
                                headingStdDev.get()),
                        VecBuilder.fill(
                                visionStdDevX.get(),
                                visionStdDevY.get(),
                                visionStdDevZ.get(),
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
    }
}
