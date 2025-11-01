/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.drive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.teamdeadbolts.constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private SwerveDriveOdometry swerveDriveOdometry;
    private SwerveModule[] modules;

    private LoggedNetworkNumber maxSpeed = new LoggedNetworkNumber("Tuning/Swerve/MaxSpeed", 1.0);

    public SwerveSubsystem() {
        this.resetGyro();
        this.modules =
                new SwerveModule[] {
                    new SwerveModule(SwerveConstants.FRONT_RIGHT_CONFIG),
                    new SwerveModule(SwerveConstants.FRONT_LEFT_CONFIG),
                    new SwerveModule(SwerveConstants.BACK_RIGHT_CONFIG),
                    new SwerveModule(SwerveConstants.BACK_LEFT_CONFIG)
                };

        this.swerveDriveOdometry =
                new SwerveDriveOdometry(
                        SwerveConstants.SWERVE_KINEMATICS, getGyroRotation(), getModulePositions());
    }

    /**
     * Make the robot drive
     * @param translation A {@link Translation2d} representing the x and y motion in <strong>m/s</strong>
     * @param rotation The target rotation speed for the motion in <strong>rads/sec</strong>
     * @param fieldRelative Weather or not to drive the robot relative to the field or itself
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] states =
                SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(),
                                        translation.getY(),
                                        rotation,
                                        getHeading())
                                : new ChassisSpeeds(
                                        translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed.get());
        for (SwerveModule m : modules) {
            m.setDesiredState(states[m.getModuleNumber()]);
        }
    }

    /**
     * Resets the gyro
     */
    public void resetGyro() {
        gyro.reset();
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule m : this.modules) {
            states[m.getModuleNumber()] = m.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule m : modules) {
            positions[m.getModuleNumber()] = m.getPosition();
        }
        return positions;
    }

    public Pose2d getPose2d() {
        return swerveDriveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveDriveOdometry.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return this.getPose2d().getRotation();
    }

    public void zeroHeading() {
        swerveDriveOdometry.resetPosition(
                getGyroRotation(),
                getModulePositions(),
                new Pose2d(this.getPose2d().getTranslation(), new Rotation2d()));
    }

    public void resetModulesToAbs() {
        for (SwerveModule m : modules) {
            m.resetToAbs();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, this.maxSpeed.get());
        for (SwerveModule m : this.modules) {
            m.setDesiredState(desiredStates[m.getModuleNumber()]);
        }
    }

    @Override
    public void periodic() {
        swerveDriveOdometry.update(getGyroRotation(), getModulePositions());
        Logger.recordOutput("States", getModuleStates());
        Logger.recordOutput("Pose", getPose2d());

        for (SwerveModule m : modules) {
            Logger.recordOutput(
                    "Module " + m.getModuleNumber() + " Rotation", m.getRotation().getDegrees());
            Logger.recordOutput(
                    "Module " + m.getModuleNumber() + " Integrated",
                    m.getPosition().angle.getDegrees());
            Logger.recordOutput(
                    "Module " + m.getModuleNumber() + " Velocity",
                    m.getState().speedMetersPerSecond);
        }
    }
}
