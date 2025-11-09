/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.teamdeadbolts.constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private SwerveDriveOdometry swerveDriveOdometry;
    private SwerveModule[] modules;
    private SlewRateLimiter slewRateLimiterTranslationalX;
    private SlewRateLimiter slewRateLimiterTranslationalY;
    private SlewRateLimiter slewRateLimiterRotaional;

    private LoggedNetworkNumber maxModuleSpeed =
            new LoggedNetworkNumber("Tuning/Swerve/MaxModuleSpeed", 1.0);
    private LoggedNetworkNumber slewRateTranslational =
            new LoggedNetworkNumber("Tuning/Swerve/TranslationSlew", 1.0);
    private LoggedNetworkNumber slewRateRotaional =
            new LoggedNetworkNumber("Tuning/Swerve/RotationSlew", 1.0);

    private SysIdRoutine driveRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            (volts) -> {
                                for (SwerveModule m : this.modules) {
                                    m.setVolts(volts.in(Volts));
                                }
                            },
                            null,
                            this));

    public SwerveSubsystem() {
        this.resetGyro();
        this.modules =
                new SwerveModule[] {
                    new SwerveModule(SwerveConstants.FRONT_LEFT_CONFIG),
                    new SwerveModule(SwerveConstants.FRONT_RIGHT_CONFIG),
                    new SwerveModule(SwerveConstants.BACK_LEFT_CONFIG),
                    new SwerveModule(SwerveConstants.BACK_RIGHT_CONFIG)
                };

        this.swerveDriveOdometry =
                new SwerveDriveOdometry(
                        SwerveConstants.SWERVE_KINEMATICS, getGyroRotation(), getModulePositions());

        this.refreshTuning();
    }

    /**
     * Make the robot drive
     * @param translation A {@link Translation2d} representing the x and y motion in <strong>m/s</strong>
     * @param rotation The target rotation speed for the motion in <strong>rads/sec</strong>
     * @param fieldRelative Weather or not to drive the robot relative to the field or itself
     */
    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean slewRates) {
        SwerveModuleState[] states =
                SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        slewRates
                                                ? slewRateLimiterTranslationalX.calculate(
                                                        translation.getX())
                                                : translation.getX(),
                                        slewRates
                                                ? slewRateLimiterTranslationalY.calculate(
                                                        translation.getY())
                                                : translation.getY(),
                                        slewRates
                                                ? slewRateLimiterRotaional.calculate(rotation)
                                                : rotation,
                                        getHeading())
                                : new ChassisSpeeds(
                                        slewRates
                                                ? slewRateLimiterTranslationalX.calculate(
                                                        translation.getX())
                                                : translation.getX(),
                                        slewRates
                                                ? slewRateLimiterTranslationalY.calculate(
                                                        translation.getY())
                                                : translation.getY(),
                                        slewRates
                                                ? slewRateLimiterRotaional.calculate(rotation)
                                                : rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxModuleSpeed.get());
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

    /**
     * Get the rotation of the robot
     * @return The rotation
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    /**
     * Get the states of all of the modules
     * @return 4 {@link SwerveModuleState}s (front left, front right, back left, back right)
     */
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, this.maxModuleSpeed.get());
        for (SwerveModule m : this.modules) {
            m.setDesiredState(desiredStates[m.getModuleNumber()]);
        }
    }

    /**
     * Refresh the tuning values from AdvantageKit
     */
    public void refreshTuning() {
        this.slewRateLimiterTranslationalX = new SlewRateLimiter(slewRateTranslational.get());
        this.slewRateLimiterTranslationalY = new SlewRateLimiter(slewRateTranslational.get());
        this.slewRateLimiterRotaional = new SlewRateLimiter(slewRateRotaional.get());

        for (SwerveModule m : modules) {
            m.configure();
        }
    }

    public SwerveModule getModule(int id) {
        return this.modules[id];
    }

    public Command runDriveQuasiTest(Direction direction) {
        return driveRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(Direction direction) {
        return driveRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        swerveDriveOdometry.update(getGyroRotation(), getModulePositions());
        for (SwerveModule m : this.modules) {
            m.tick();
        }
        Logger.recordOutput("States", getModuleStates());
        Logger.recordOutput("Pose", getPose2d());
    }
}
