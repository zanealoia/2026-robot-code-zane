/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private SavedLoggedNetworkNumber tMaxVel =
            new SavedLoggedNetworkNumber("Tuning/DriveToPoint/Translation/MaxVelMPS", 0.0);

    private SavedLoggedNetworkNumber tMaxAcc =
            new SavedLoggedNetworkNumber("Tuning/DriveToPoint/Translation/MaxAccMPS", 0.0);
    private SavedLoggedNetworkNumber tAdvance =
            new SavedLoggedNetworkNumber("Tuning/DriveToPoint/Translation/StepSize", 0.0);

    /** Rotation tuning values */
    private SavedLoggedNetworkNumber rMaxVel =
            new SavedLoggedNetworkNumber("Tuning/DriveToPoint/Rotation/MaxVelDPS", 0.0);

    private SavedLoggedNetworkNumber rMaxAcc =
            new SavedLoggedNetworkNumber("Tuning/DriveToPoint/Rotation/MaxAccDPS", 0.0);
    private SavedLoggedNetworkNumber rAdvance =
            new SavedLoggedNetworkNumber("Tuning/DriveToPoint/Rotation/StepSize", 0.0);

    private TrapezoidProfile tProfile =
            new TrapezoidProfile(new Constraints(tMaxVel.get(), tMaxAcc.get()));

    private TrapezoidProfile rProfile =
            new TrapezoidProfile(new Constraints(rMaxVel.get(), rMaxAcc.get()));

    private Pose2d target;
    private Pose2d tolerance;

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
        tProfile = new TrapezoidProfile(new Constraints(tMaxVel.get(), tMaxAcc.get()));
        rProfile =
                new TrapezoidProfile(
                        new Constraints(
                                Units.degreesToRadians(rMaxVel.get()),
                                Units.degreesToRadians(rMaxAcc.get())));
    }

    @Override
    public void execute() {
        Pose2d currentPose = robotState.getRobotPose().toPose2d();
        double deltaX = target.getX() - currentPose.getX();
        double deltaY = target.getY() - currentPose.getY();
        double distance = Math.hypot(deltaX, deltaY);

        double tTrapCalc =
                -this.tProfile.calculate(
                                tAdvance.get(),
                                new TrapezoidProfile.State(
                                        distance,
                                        -Math.hypot(
                                                this.robotState.getRobotVelocities()
                                                        .vxMetersPerSecond,
                                                this.robotState.getRobotVelocities()
                                                        .vyMetersPerSecond)),
                                new TrapezoidProfile.State(0.0, 0.0))
                        .velocity;

        double angle = Math.atan2(deltaY, deltaX);
        double xVel = tTrapCalc * Math.cos(angle);
        double yVel = tTrapCalc * Math.sin(angle);

        double rotDistance =
                this.target.getRotation().minus(currentPose.getRotation()).getRadians();

        // double rotGoal =
        //         MathUtil.inputModulus(
        //                 target.getRotation().getRadians() -
        // currentPose.getRotation().getRadians(),
        //                 -Math.PI,
        //                 Math.PI);
        double rotTrapCalc =
                this.rProfile.calculate(
                                rAdvance.get(),
                                new TrapezoidProfile.State(
                                        rotDistance,
                                        this.robotState.getRobotVelocities().omegaRadiansPerSecond),
                                new TrapezoidProfile.State(0.0, 0.0))
                        .velocity;

        swerveSubsystem.drive(new Translation2d(xVel, yVel), rotTrapCalc, true, false);

        Logger.recordOutput("DriveToPoint/TargetPose", target);
        Logger.recordOutput("DriveToPoint/DistanceToTarget", distance);
        Logger.recordOutput("DriveToPoint/XVelCmd", xVel);
        Logger.recordOutput("DriveToPoint/YVelCmd", yVel);
        Logger.recordOutput("DriveToPoint/RotVelCmd", rotTrapCalc);

        Logger.recordOutput("DriveToPoint/RotError", Units.radiansToDegrees(rotDistance));
        // Logger.recordOutput("DriveToPoint/RotMeasurement", );
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = robotState.getRobotPose().toPose2d();
        boolean withinX = Math.abs(currentPose.getX() - target.getX()) <= tolerance.getX();
        boolean withinY = Math.abs(currentPose.getY() - target.getY()) <= tolerance.getY();
        boolean withinRotation =
                Math.abs(currentPose.getRotation().getDegrees() - target.getRotation().getDegrees())
                        <= tolerance.getRotation().getDegrees();
        return withinX && withinY && withinRotation;
    }
}
