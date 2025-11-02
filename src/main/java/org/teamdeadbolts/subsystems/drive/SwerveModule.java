/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.drive;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.utils.CtreUtils;
import org.teamdeadbolts.utils.MathUtils;

public class SwerveModule {
    private int moduleNumber;
    private Rotation2d offset;

    private TalonFX driveMotor;
    private TalonFX turningMotor;
    private CANcoder encoder;

    private final VelocityVoltage driveVel = new VelocityVoltage(0);

    private final PositionVoltage turningPosition = new PositionVoltage(0);

    private final SimpleMotorFeedforward driveFF =
            new SimpleMotorFeedforward(
                    new LoggedNetworkNumber("Tuning/Swerve/Drive/FeedforwardKS", 0.0).get(),
                    new LoggedNetworkNumber("Tuning/Swerve/Drive/FeedforwardKV", 0.0).get());

    /**
     * Create a new swerve module
     * @param config The {@link SwerveModuleConfig} for the module
     */
    public SwerveModule(SwerveModuleConfig config) {
        this.offset = config.offset();
        this.moduleNumber = config.moduleNumber();

        this.encoder = new CANcoder(config.encoderId());
        this.driveMotor = new TalonFX(config.driveMotorId());
        this.turningMotor = new TalonFX(config.turningMotorId());
        this.resetToAbs();
        this.configure();
        this.driveMotor.setPosition(0.0);
    }

    /**
     * Configure the drive and turning motors as described in {@link CtreUtils}
     */
    public void configure() {
        this.driveMotor.getConfigurator().apply(CtreUtils.swerveDriveFXConfig);
        this.turningMotor.getConfigurator().apply(CtreUtils.swerveTurningFXConfig);
    }

    /**
     * Set the desired state for the module
     * @param desiredState The desired state
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState.optimize(getRotation());
        this.setSpeed(desiredState.speedMetersPerSecond);
        this.setAngle(desiredState.angle);
    }

    /**
     * Set the speed of the module (drive)
     * @param speed The desired speed in <strong>m/s</strong>
     */
    private void setSpeed(double speed) {
        Logger.recordOutput("Swerve/Module " + moduleNumber + "/TargetSpeed", speed);
        driveVel.Velocity = MathUtils.MPSToRPS(speed, SwerveConstants.WHEEL_CIRCUMFERENCE);
        driveVel.FeedForward = driveFF.calculate(speed);
        driveMotor.setControl(this.driveVel);
    }

    /**
     * Set the angle
     * @param angle The angle as a {@link Rotation2d}
     */
    private void setAngle(Rotation2d angle) {
        Logger.recordOutput("Swerve/Module " + moduleNumber + "/TargetAngle", angle.getDegrees());
        turningMotor.setControl(this.turningPosition.withPosition(angle.getRotations()));
    }

    /**
     * Get the current rotation of the module
     * @return The rotation of the module
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble())
                .plus(this.offset);
    }

    /**
     * Reset the module to the absoulte position
     */
    public void resetToAbs() {
        double absPos = this.getRotation().getRotations() + offset.getRotations();
        turningMotor.setPosition(absPos);
    }

    /**
     * Get the current state of the module
     * @return The state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                MathUtils.RPSToMPS(
                        driveMotor.getVelocity().getValueAsDouble(),
                        SwerveConstants.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(turningMotor.getPosition().getValueAsDouble()));
    }

    /**
     * Get the current position of the module
     * @return The position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                MathUtils.RPSToMPS(
                        driveMotor.getVelocity().getValueAsDouble(),
                        SwerveConstants.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(turningMotor.getPosition().getValueAsDouble()));
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    /**
     * A config for a module
     * @param offset The rotational offset from zero for the module
     * @param driveMoterId The CAN id of the drive motor
     * @param turningMotorId The CAN id of the turning motor
     * @param encoderId The CAN id of the encoder
     */
    public record SwerveModuleConfig(
            int moduleNumber,
            Rotation2d offset,
            int driveMotorId,
            int turningMotorId,
            int encoderId) {}
}
