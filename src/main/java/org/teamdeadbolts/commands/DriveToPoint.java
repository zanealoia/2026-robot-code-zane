/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    /** Cross track */
    private SavedLoggedNetworkNumber cP =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Cross/kP", 0.0);

    private SavedLoggedNetworkNumber cI =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Cross/kI", 0.0);
    private SavedLoggedNetworkNumber cD =
            SavedLoggedNetworkNumber.get("Tuning/DriveToPoint/Cross/kD", 0.0);

    private final ProfiledPIDController tController =
            new ProfiledPIDController(tP.get(), tI.get(), tD.get(), new Constraints(0, 0));

    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(rP.get(), rI.get(), rD.get(), new Constraints(0, 0));

    private final PIDController lateralController = new PIDController(cP.get(), cI.get(), cD.get());

    private Pose2d target;
    private Pose2d tolerance;

    private Translation2d startPoint;
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
        /** Refresh pid values from nt */
        Constraints transConstraints = new Constraints(tMaxVel.get(), tMaxAcc.get());
        Constraints rotConstraints =
                new Constraints(
                        Units.degreesToRadians(rMaxVel.get()),
                        Units.degreesToRadians(rMaxAcc.get()));

        tController.setConstraints(transConstraints);
        tController.setPID(tP.get(), tI.get(), tD.get());

        thetaController.setConstraints(rotConstraints);
        thetaController.setPID(rP.get(), rI.get(), rD.get());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        lateralController.setPID(cP.get(), cI.get(), cD.get());

        Pose2d currentPose = this.robotState.getRobotPose().toPose2d();

        startPoint = currentPose.getTranslation();
        totalDistance = startPoint.getDistance(target.getTranslation());

        Rotation2d pathHeading = target.getTranslation().minus(startPoint).getAngle();

        ChassisSpeeds speeds = this.robotState.getRobotVelocities();
        double velAlongPath =
                speeds.vxMetersPerSecond * pathHeading.getCos()
                        + speeds.vyMetersPerSecond * pathHeading.getSin();

        tController.reset(0, velAlongPath);
        thetaController.reset(currentPose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);
        lateralController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = this.robotState.getRobotPose().toPose2d();
        Translation2d currOffset = currentPose.getTranslation().minus(startPoint);

        Rotation2d pathHeading = target.getTranslation().minus(startPoint).getAngle();

        double progress =
                currOffset.getX() * pathHeading.getCos() + currOffset.getY() * pathHeading.getSin();

        double crossTrackError =
                -currOffset.getX() * pathHeading.getSin()
                        + currOffset.getY() * pathHeading.getCos();

        double forwardVel = tController.calculate(progress, totalDistance);
        double lateralVel = lateralController.calculate(crossTrackError, 0);

        double xVel = forwardVel * pathHeading.getCos() - lateralVel * pathHeading.getSin();
        double yVel = forwardVel * pathHeading.getSin() + lateralVel * pathHeading.getCos();

        double thetaVel =
                thetaController.calculate(
                        currentPose.getRotation().getRadians(), target.getRotation().getRadians());

        swerveSubsystem.drive(new Translation2d(xVel, yVel), thetaVel, true, false);

        Logger.recordOutput("DriveToPoint/TargetPose", target);
        Logger.recordOutput("DriveToPoint/XVelCmd", xVel);
        Logger.recordOutput("DriveToPoint/YVelCmd", yVel);
        Logger.recordOutput("DriveToPoint/RotVelCmd", Units.radiansToDegrees(thetaVel));
        Logger.recordOutput("DriveToPoint/LateralVelCmd", lateralVel);
        Logger.recordOutput("DriveToPoint/ForwardVelCmd", forwardVel);
        Logger.recordOutput("DriveToPoint/Progress", progress);
        Logger.recordOutput("DriveToPoint/TransError", tController.getPositionError());
        Logger.recordOutput("DriveToPoint/CrossTrackError", crossTrackError);
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
