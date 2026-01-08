/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.state.RobotState;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class DriveToPoint extends Command {
    private SwerveSubsystem swerveSubsystem;

    private RobotState robotState = RobotState.getInstance();

    /** Translation tuning values */
    private SavedLoggedNetworkNumber tP =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Translation/kP", 0.0);

    private SavedLoggedNetworkNumber tI =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Translation/kI", 0.0);
    private SavedLoggedNetworkNumber tD =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Translation/kD", 0.0);
    private SavedLoggedNetworkNumber tMaxVel =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Translation/MaxVelMPS", 0.0);

    private SavedLoggedNetworkNumber tMaxAcc =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Translation/MaxAccMPS", 0.0);

    /** Rotation tuning values */
    private SavedLoggedNetworkNumber rP =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Rotation/kP", 0.0);

    private SavedLoggedNetworkNumber rI =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Rotation/kI", 0.0);
    private SavedLoggedNetworkNumber rD =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Rotation/kD", 0.0);
    private SavedLoggedNetworkNumber rMaxVel =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Rotation/MaxVelDPS", 0.0);

    private SavedLoggedNetworkNumber rMaxAcc =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Rotation/MaxAccDPS", 0.0);

    private final ProfiledPIDController tController =
            new ProfiledPIDController(tP.get(), tI.get(), tD.get(), new Constraints(0, 0));

    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(rP.get(), rI.get(), rD.get(), new Constraints(0, 0));


    private Pose2d target;
    private Pose2d tolerance;

    double totalDistance;

    /**
     * Command to drive to a point
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param target The target pose
     * @param tolerance The tolerance for each axis
     */
    public DriveToPoint(SwerveSubsystem swerveSubsystem, Pose2d target, Pose2d tolerance) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;
        this.tolerance = tolerance;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Constraints transConstraints = new Constraints(tMaxVel.get(), tMaxAcc.get());
        tController.setConstraints(transConstraints);
        tController.setPID(tP.get(), tI.get(), tD.get());

        thetaController.setConstraints(
                new Constraints(
                        Units.degreesToRadians(rMaxVel.get()),
                        Units.degreesToRadians(rMaxAcc.get())));
        thetaController.setPID(rP.get(), rI.get(), rD.get());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d currentPose = this.robotState.getRobotPose().toPose2d();
        ChassisSpeeds speeds = this.robotState.getRobotVelocities();

        double currentDistance = currentPose.getTranslation().getDistance(target.getTranslation());

        Translation2d unitVectorToTarget =
                target.getTranslation().minus(currentPose.getTranslation()).div(currentDistance);
        double velocityMagnitude =
                speeds.vxMetersPerSecond * unitVectorToTarget.getX()
                        + speeds.vyMetersPerSecond * unitVectorToTarget.getY();

        tController.reset(currentDistance, velocityMagnitude);
        thetaController.reset(currentPose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        Pose2d currentPose = this.robotState.getRobotPose().toPose2d();
        Translation2d translationError =
                target.getTranslation().minus(currentPose.getTranslation());
        double distance = translationError.getNorm();

        double totalVel = -tController.calculate(distance, 0);

        double xVel = (translationError.getX() / distance) * totalVel;
        double yVel = (translationError.getY() / distance) * totalVel;

        double thetaVel =
                thetaController.calculate(
                        currentPose.getRotation().getRadians(), target.getRotation().getRadians());

        swerveSubsystem.drive(new Translation2d(xVel, yVel), thetaVel, true, false, true);

        Logger.recordOutput("DriveToPoint/TargetPose", target);
        Logger.recordOutput("DriveToPoint/XVelCmd", xVel);
        Logger.recordOutput("DriveToPoint/YVelCmd", yVel);
        Logger.recordOutput("DriveToPoint/RotVelCmd", Units.radiansToDegrees(thetaVel));
        Logger.recordOutput("DriveToPoint/TransError", tController.getPositionError());
        Logger.recordOutput(
                "DriveToPoint/ThetaError",
                Units.radiansToDegrees(thetaController.getPositionError()));
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = robotState.getRobotPose().toPose2d();
        ChassisSpeeds speeds = robotState.getRobotVelocities();

        double distToTarget = currentPose.getTranslation().getDistance(target.getTranslation());
        boolean atTranslation = distToTarget <= tolerance.getTranslation().getNorm();

        double angleError =
                Math.abs(currentPose.getRotation().minus(target.getRotation()).getDegrees());
        boolean atRotation = angleError <= tolerance.getRotation().getDegrees();

        boolean isSettled = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1;

        return (tController.atGoal() || atTranslation) && atRotation && isSettled;
    }
}
