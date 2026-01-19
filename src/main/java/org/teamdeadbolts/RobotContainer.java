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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    @SuppressWarnings("unused")
    private VisionSubsystem visionSubsystem =
            new VisionSubsystem(swerveSubsystem, new PhotonVisionIO("CenterCam", new Transform3d()));

    // private HopperSubsystem hopperSubsystem = new HopperSubsystem();
    // private IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    // private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    // private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private CommandXboxController primaryController = new CommandXboxController(0);

    private RobotState robotState = RobotState.getInstance();

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private SavedLoggedNetworkNumber controllerDeadband =
            SavedLoggedNetworkNumber.get("Tuning/Drive/ControllerDeadband", 0.08);

    private SavedLoggedNetworkNumber maxRobotSpeed = SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotSpeed", 1.0);

    private SavedLoggedNetworkNumber maxRobotAnglarSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotAngluarSpeed", 1.0);

    public RobotContainer() {
        robotState.initPoseEstimator(
                new Rotation3d(swerveSubsystem.getGyroRotation()), swerveSubsystem.getModulePositions());
        // RobotConfig robotConfig = new RobotConfig(
        //         Mass.ofBaseUnits(30, Pounds),
        //         MomentOfInertia.ofBaseUnits(5, KilogramSquareMeters),
        //         new ModuleConfig(
        //                 SwerveConstants.WHEEL_CIRCUMFERENCE / (2 * Math.PI),
        //                 5,
        //                 1,
        //                 new DCMotor(0, 0, 0, 0, 0, 0),
        //                 120,
        //                 4),
        //         new Translation2d[] {});
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                () -> robotState.getRobotPose().toPose2d(),
                (pose2d) -> robotState.setEstimatedPose(new Pose3d(pose2d)),
                robotState::getRobotRelativeRobotVelocities,
                (speeds) -> swerveSubsystem.drive(speeds, false, false, false),
                new PPHolonomicDriveController(new PIDConstants(0), new PIDConstants(0)),
                config,
                () -> {
                    return false;
                },
                this.swerveSubsystem);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", autoChooser);
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

        // shooterSubsystem.setDefaultCommand(new DefaultShooterCommand(shooterSubsystem, indexerSubsystem));
        // hopperSubsystem.setDefaultCommand(
        //         new RunCommand(() -> hopperSubsystem.setState(HopperSubsystem.State.HOLD), hopperSubsystem));
        // intakeSubsystem.setDefaultCommand(
        //         new RunCommand(() -> intakeSubsystem.setState(IntakeSubsystem.State.STOWED), intakeSubsystem));
        // indexerSubsystem.setDefaultCommand(
        // new RunCommand(() -> indexerSubsystem.setState(IndexerSubsystem.State.OFF), indexerSubsystem));

        primaryController
                .a()
                .whileTrue(new RunCommand(
                        () -> {
                            // CtreConfigs.init();
                            swerveSubsystem.refreshTuning(false);
                        },
                        swerveSubsystem));

        primaryController.x().whileTrue(new RunCommand(() -> swerveSubsystem.resetGyro(), swerveSubsystem));

        primaryController
                .y()
                .whileTrue(new RunCommand(() -> robotState.setEstimatedPose(new Pose3d()), swerveSubsystem));

        primaryController.povRight().whileTrue(swerveSubsystem.runDriveDynamTest(Direction.kReverse));
        primaryController.povDown().whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kForward));
        primaryController.povLeft().whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
