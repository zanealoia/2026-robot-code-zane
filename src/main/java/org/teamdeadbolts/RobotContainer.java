/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.teamdeadbolts.commands.DriveCommand;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.utils.CtreConfigs;

public class RobotContainer {

    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private CommandXboxController primaryController = new CommandXboxController(0);

    private LoggedNetworkNumber controllerDeadband =
            new LoggedNetworkNumber("Tuning/Drive/ControllerDeadband", 0.08);

    private LoggedNetworkNumber maxRobotSpeed =
            new LoggedNetworkNumber("Tuning/Drive/MaxRobotSpeed", 1.0);

    private LoggedNetworkNumber maxRobotAnglarSpeed =
            new LoggedNetworkNumber("Tuning/Drive/MaxRobotAngluarSpeed", 1.0);

    public RobotContainer() {
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
                                                primaryController.getRightX(),
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
                .whileTrue(
                        new RunCommand(() -> swerveSubsystem.resetModulesToAbs(), swerveSubsystem));
        primaryController
                .x()
                .whileTrue(new RunCommand(() -> swerveSubsystem.resetGyro(), swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
