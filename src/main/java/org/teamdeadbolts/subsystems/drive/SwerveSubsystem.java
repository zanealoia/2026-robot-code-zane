/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.state.RobotState;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private SwerveModule[] modules;
    private SlewRateLimiter slewRateLimiterTranslationalX;
    private SlewRateLimiter slewRateLimiterTranslationalY;
    private SlewRateLimiter slewRateLimiterRotaional;

    private SavedLoggedNetworkNumber maxModuleSpeed =
            new SavedLoggedNetworkNumber("Tuning/Swerve/MaxModuleSpeed", 1.0);
    private SavedLoggedNetworkNumber slewRateTranslational =
            new SavedLoggedNetworkNumber("Tuning/Swerve/TranslationSlew", 1.0);
    private SavedLoggedNetworkNumber slewRateRotaional =
            new SavedLoggedNetworkNumber("Tuning/Swerve/RotationSlew", 1.0);

    private SysIdRoutine driveRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            (volts) -> {
                                for (SwerveModule m : this.modules) {
                                    m.setVolts(volts.in(Volts));
                                    if (m.getModuleNumber() % 2 == 0)
                                        m.setAngle(Rotation2d.fromDegrees(0));
                                    else m.setAngle(Rotation2d.fromDegrees(180));
                                }
                            },
                            (log) -> {
                                for (SwerveModule m : this.modules) {
                                    log.motor(getName() + "-" + m.getModuleNumber())
                                            .linearVelocity(
                                                    LinearVelocity.ofBaseUnits(
                                                            m.getState().speedMetersPerSecond,
                                                            MetersPerSecond));
                                }
                            },
                            this));

    private SysIdRoutine turnRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            (volts) -> {
                                this.modules[0].setTurnVolts(volts.in(Volts));
                            },
                            (log) -> {
                                SwerveModule m = modules[0];
                                log.motor(getName() + "-" + m.getModuleNumber())
                                        .angularPosition(
                                                Angle.ofBaseUnits(
                                                        m.getPosition().angle.getDegrees(),
                                                        Degrees));

                                log.motor(getName() + "-" + m.getModuleNumber())
                                        .angularVelocity(
                                                AngularVelocity.ofBaseUnits(
                                                        m.getTurnVelocity(), RotationsPerSecond));
                                log.motor(getName() + "-" + m.getModuleNumber())
                                        .voltage(Voltage.ofBaseUnits(m.getTurnVoltage(), Volts));
                                // log.motor(getName() + "-" +
                                // m.getModuleNumber()).angularVelocity(AngularVelocity.ofBaseUnits(m.getState().angle, null))

                            },
                            this));

    /* Callback that the swerve subsystem will update with module positions and gyro rotation */
    private BiConsumer<SwerveModulePosition[], Rotation2d> modulePositionCallback = null;

    public SwerveSubsystem() {
        this.resetGyro();
        this.modules =
                new SwerveModule[] {
                    new SwerveModule(SwerveConstants.FRONT_LEFT_CONFIG),
                    new SwerveModule(SwerveConstants.FRONT_RIGHT_CONFIG),
                    new SwerveModule(SwerveConstants.BACK_LEFT_CONFIG),
                    new SwerveModule(SwerveConstants.BACK_RIGHT_CONFIG)
                };

        this.refreshTuning();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Make the robot drive
     * @param translation A {@link Translation2d} representing the x and y motion in <strong>m/s</strong>
     * @param rotation The target rotation speed for the motion in <strong>rads/sec</strong>
     * @param fieldRelative Weather or not to drive the robot relative to the field or itself
     */
    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean slewRates) {
        Logger.recordOutput("Swerve/CommandedVelocitiesTrans", translation);
        Logger.recordOutput("Swerve/CommandedVelocitiesRot", Units.radiansToDegrees(rotation));

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
                                        getGyroRotation())
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
        return Rotation2d.fromDegrees(gyro.getAngle()).unaryMinus();
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

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return SwerveConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        ChassisSpeeds robotRelative = this.getRobotRelativeChassisSpeeds();
        return new ChassisSpeeds(
                robotRelative.vxMetersPerSecond * Math.cos(getGyroRotation().getRadians())
                        - robotRelative.vyMetersPerSecond
                                * Math.sin(getGyroRotation().getRadians()),
                robotRelative.vyMetersPerSecond * Math.cos(getGyroRotation().getRadians())
                        + robotRelative.vxMetersPerSecond
                                * Math.sin(getGyroRotation().getRadians()),
                robotRelative.omegaRadiansPerSecond);
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

    public Command runTurnQuasiTest(Direction direction) {
        return turnRoutine.quasistatic(direction);
    }

    public Command runTurnDynamTest(Direction direction) {
        return turnRoutine.dynamic(direction);
    }

    public void setModulePositionCallback(BiConsumer<SwerveModulePosition[], Rotation2d> callback) {
        this.modulePositionCallback = callback;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Swerve/GyroRotationDeg", getGyroRotation().getDegrees());
        if (this.modulePositionCallback != null)
            this.modulePositionCallback.accept(getModulePositions(), getGyroRotation());
        for (SwerveModule m : this.modules) {
            m.tick();
        }

        ChassisSpeeds speeds = getFieldRelativeChassisSpeeds();
        RobotState.getInstance().setRobotVelocities(speeds);
        Logger.recordOutput("Swerve/RobotVelocities", speeds);
    }
}
