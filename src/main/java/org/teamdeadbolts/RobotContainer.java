/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.teamdeadbolts.commands.DriveCommand;
import org.teamdeadbolts.commands.DriveToPoint;
import org.teamdeadbolts.state.PoseEstimator;
import org.teamdeadbolts.state.vision.VisionIOPhoton;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class RobotContainer {

    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private CommandXboxController primaryController = new CommandXboxController(0);

    private PoseEstimator poseEstimator =
            new PoseEstimator(swerveSubsystem, new VisionIOPhoton("CenterCam", new Transform3d()));

    private SavedLoggedNetworkNumber controllerDeadband =
            SavedLoggedNetworkNumber.get("Tuning/Drive/ControllerDeadband", 0.08);

    private SavedLoggedNetworkNumber maxRobotSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotSpeed", 1.0);

    private SavedLoggedNetworkNumber maxRobotAnglarSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotAngluarSpeed", 1.0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Xbox controllers push "up" = neg valve so invert everything
        swerveSubsystem.setDefaultCommand(
                new DriveCommand(
                        swerveSubsystem,
                        () ->
                                -MathUtil.applyDeadband(
                                                primaryController.getLeftY(),
                                                controllerDeadband.get())
                                        * maxRobotSpeed.get(),
                        () ->
                                -MathUtil.applyDeadband(
                                                primaryController.getLeftX(),
                                                controllerDeadband.get())
                                        * maxRobotSpeed.get(),
                        () ->
                                -MathUtil.applyDeadband(
                                                primaryController.getRightX(),
                                                controllerDeadband.get())
                                        * Math.toRadians(maxRobotAnglarSpeed.get()),
                        true));

        primaryController
                .a()
                .whileTrue(
                        new RunCommand(
                                () -> {
                                    // CtreConfigs.init();
                                    swerveSubsystem.refreshTuning(false);
                                },
                                swerveSubsystem));

        primaryController
                .b()
                .whileTrue(
                        new DriveToPoint(
                                swerveSubsystem,
                                new Pose2d(
                                        new Translation2d(15.86, 1.93),
                                        Rotation2d.fromDegrees(-56)),
                                new Pose2d(
                                        new Translation2d(0.01, 0.01), Rotation2d.fromDegrees(1))));
        primaryController
                .x()
                .whileTrue(new RunCommand(() -> swerveSubsystem.resetGyro(), swerveSubsystem));

        primaryController
                .y()
                .whileTrue(
                        new RunCommand(
                                () -> poseEstimator.setPosition(new Pose3d()), swerveSubsystem));

        primaryController.povUp().whileTrue(swerveSubsystem.runDriveDynamTest(Direction.kForward));
        primaryController
                .povRight()
                .whileTrue(swerveSubsystem.runDriveDynamTest(Direction.kReverse));
        primaryController
                .povDown()
                .whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kForward));
        primaryController
                .povLeft()
                .whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void periodic() {
        poseEstimator.update();
    }
}
