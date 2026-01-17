/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.teamdeadbolts.commands.DriveCommand;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.vision.PhotonVisionIO;
import org.teamdeadbolts.subsystems.vision.VisionSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class RobotContainer {

    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private CommandXboxController primaryController = new CommandXboxController(0);

    private RobotState robotState = RobotState.getInstance();

    private VisionSubsystem visionSubsystem =
            new VisionSubsystem(swerveSubsystem, new PhotonVisionIO("CenterCam", new Transform3d()));
    private SavedLoggedNetworkNumber controllerDeadband =
            SavedLoggedNetworkNumber.get("Tuning/Drive/ControllerDeadband", 0.08);

    private SavedLoggedNetworkNumber maxRobotSpeed = SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotSpeed", 1.0);

    private SavedLoggedNetworkNumber maxRobotAnglarSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotAngluarSpeed", 1.0);

    public RobotContainer() {
        robotState.initPoseEstimator(new Rotation3d(swerveSubsystem.getGyroRotation()), swerveSubsystem.getModulePositions());
        RobotConfig robotConfig = null;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                () -> robotState.getRobotPose().toPose2d(),
                (pose2d) -> robotState.setEstimatedPose(new Pose3d(pose2d)),
                robotState::getRobotRelativeRobotVelocities,
                (speeds) -> swerveSubsystem.drive(speeds, false, false, false),
                new PPHolonomicDriveController(new PIDConstants(0), new PIDConstants(0)),
                robotConfig,
                () -> {
                    return false;
                },
                this.swerveSubsystem);

        configureBindings();
    }

    private void configureBindings() {
        // Xbox controllers push "up" = neg valve so invert everything
        swerveSubsystem.setDefaultCommand(new DriveCommand(
                swerveSubsystem,
                () -> -MathUtil.applyDeadband(primaryController.getLeftY(), controllerDeadband.get())
                        * maxRobotSpeed.get(),
                () -> -MathUtil.applyDeadband(primaryController.getLeftX(), controllerDeadband.get())
                        * maxRobotSpeed.get(),
                () -> -MathUtil.applyDeadband(primaryController.getRightX(), controllerDeadband.get())
                        * Math.toRadians(maxRobotAnglarSpeed.get()),
                true));

        primaryController
                .a()
                .whileTrue(new RunCommand(
                        () -> {
                            // CtreConfigs.init();
                            swerveSubsystem.refreshTuning(false);
                        },
                        swerveSubsystem));

        primaryController.x().whileTrue(new RunCommand(() -> swerveSubsystem.resetGyro(), swerveSubsystem));

        primaryController.y().whileTrue(new RunCommand(() -> robotState.setEstimatedPose(new Pose3d()), swerveSubsystem));

        primaryController.povUp().whileTrue(swerveSubsystem.runDriveDynamTest(Direction.kForward));
        primaryController.povRight().whileTrue(swerveSubsystem.runDriveDynamTest(Direction.kReverse));
        primaryController.povDown().whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kForward));
        primaryController.povLeft().whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
