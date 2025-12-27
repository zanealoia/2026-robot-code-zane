/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.teamdeadbolts.commands.DriveCommand;
import org.teamdeadbolts.commands.DriveToPoint;
import org.teamdeadbolts.state.PoseEstimator;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.utils.CtreConfigs;
import org.teamdeadbolts.utils.PhotonCameraWrapper;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class RobotContainer {

    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private CommandXboxController primaryController = new CommandXboxController(0);

    private PoseEstimator poseEstimator = new PoseEstimator(swerveSubsystem);

    private SavedLoggedNetworkNumber controllerDeadband =
            new SavedLoggedNetworkNumber("Tuning/Drive/ControllerDeadband", 0.08);

    private SavedLoggedNetworkNumber maxRobotSpeed =
            new SavedLoggedNetworkNumber("Tuning/Drive/MaxRobotSpeed", 1.0);

    private SavedLoggedNetworkNumber maxRobotAnglarSpeed =
            new SavedLoggedNetworkNumber("Tuning/Drive/MaxRobotAngluarSpeed", 1.0);

    private SavedLoggedNetworkNumber testVolts =
            new SavedLoggedNetworkNumber("Tuning/Drive/TestVolts", 0.0);

    public RobotContainer() {
        poseEstimator.addCamera(new PhotonCameraWrapper("CenterCam", new Transform3d()));
        configureBindings();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(
                new DriveCommand(
                        swerveSubsystem,
                        () ->
                                MathUtil.applyDeadband(
                                                primaryController.getLeftY(),
                                                controllerDeadband.get())
                                        * maxRobotSpeed.get(),
                        () ->
                                -MathUtil.applyDeadband(
                                                primaryController.getLeftX(),
                                                controllerDeadband.get())
                                        * maxRobotSpeed.get(),
                        () ->
                                MathUtil.applyDeadband(
                                                -primaryController.getRightX(),
                                                controllerDeadband.get())
                                        * Math.toRadians(maxRobotAnglarSpeed.get()),
                        true));

        primaryController
                .a()
                .whileTrue(
                        new RunCommand(
                                () -> {
                                    CtreConfigs.init();
                                    swerveSubsystem.refreshTuning();
                                },
                                swerveSubsystem));

        primaryController
                .b()
                .whileTrue(new DriveToPoint(swerveSubsystem, new Pose2d(), new Pose2d()));
        primaryController
                .x()
                .whileTrue(new RunCommand(() -> swerveSubsystem.resetGyro(), swerveSubsystem));

        primaryController
                .y()
                .whileTrue(
                        new RunCommand(
                                () -> poseEstimator.setPosition(new Pose3d()), swerveSubsystem));

        primaryController.povUp().whileTrue(swerveSubsystem.runTurnDynamTest(Direction.kForward));
        primaryController
                .povRight()
                .whileTrue(swerveSubsystem.runTurnQuasiTest(Direction.kForward));
        primaryController.povDown().whileTrue(swerveSubsystem.runTurnDynamTest(Direction.kReverse));
        primaryController.povLeft().whileTrue(swerveSubsystem.runTurnQuasiTest(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void periodic() {
        poseEstimator.update();
    }
}
