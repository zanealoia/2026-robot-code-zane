/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.utils.CtreConfigs;
import org.teamdeadbolts.utils.MathUtils;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class SwerveModule {
    private int moduleNumber;
    private Rotation2d offset;

    private TalonFX driveMotor;
    private TalonFX turningMotor;
    private CANcoder encoder;

    /** Tuning values */
    private final SavedLoggedNetworkNumber dFFkS =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Drive/FeedforwardKS", 0.0);

    private final SavedLoggedNetworkNumber dFFkV =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Drive/FeedforwardKV", 0.0);
    private final SimpleMotorFeedforward driveFF =
            new SimpleMotorFeedforward(dFFkS.get(), dFFkS.get());

    private final SavedLoggedNetworkNumber tFFkS =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Turn/FeedforwardKS", 0.0);
    private final SavedLoggedNetworkNumber tFFkV =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Turn/FeedforwardKV", 0.0);
    private final SimpleMotorFeedforward turnFF =
            new SimpleMotorFeedforward(tFFkS.get(), tFFkV.get());

    private static final SavedLoggedNetworkNumber dP =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Drive/kP", 0.0);
    private static final SavedLoggedNetworkNumber dI =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Drive/kI", 0.0);
    private static final SavedLoggedNetworkNumber dD =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Drive/kD", 0.0);

    private static final SavedLoggedNetworkNumber tP =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Turn/kP", 0.0);
    private static final SavedLoggedNetworkNumber tI =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Turn/kI", 0.0);
    private static final SavedLoggedNetworkNumber tD =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Turn/kD", 0.0);
    private static final SavedLoggedNetworkNumber tMaxVel =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Turn/MaxVelocity", 0.0);
    private static final SavedLoggedNetworkNumber tMaxAccel =
            new SavedLoggedNetworkNumber("Tuning/Swerve/Turn/MaxAcceleration", 0.0);

    /* PID */
    private ProfiledPIDController tProfiledPIDController =
            new ProfiledPIDController(
                    tP.get(), tI.get(), tD.get(), new Constraints(tMaxVel.get(), tMaxAccel.get()));

    private PIDController dPIDController = new PIDController(dP.get(), dI.get(), dD.get());

    private double targetSpeedMps = 0.0;
    private Rotation2d targetAngle = new Rotation2d();

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
        this.configure();
        this.resetToAbs();
        this.driveMotor.setPosition(0.0);
    }

    /**
     * Update motor and PID configurations from NetworkTables
     */
    public void configure() {
        this.driveMotor.getConfigurator().apply(CtreConfigs.swerveDriveFXConfig);
        this.turningMotor.getConfigurator().apply(CtreConfigs.swerveTurningFXConfig);
        this.encoder.getConfigurator().apply(CtreConfigs.swerveCANcoderConfiguration);

        this.driveFF.setKs(dFFkS.get());
        this.driveFF.setKv(dFFkV.get());

        this.turnFF.setKs(tFFkS.get());
        this.turnFF.setKv(tFFkV.get());

        this.tProfiledPIDController.setConstraints(new Constraints(tMaxVel.get(), tMaxAccel.get()));
        this.tProfiledPIDController.setPID(tP.get(), tI.get(), tD.get());
        this.tProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.dPIDController.setPID(dP.get(), dI.get(), dD.get());
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
        this.targetSpeedMps = speed;
    }

    /**
     * Set the angle
     * @param angle The angle as a {@link Rotation2d}
     */
    private void setAngle(Rotation2d angle) {
        Logger.recordOutput("Swerve/Module " + moduleNumber + "/TargetAngle", angle.getDegrees());
        Logger.recordOutput(
                "Swerve/Module " + moduleNumber + "/TargetAngleRaw",
                angle.minus(offset).getDegrees());

        this.targetAngle = angle;
    }
    /**
     * Get the current rotation of the module
     * @return The rotation of the module
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(
                encoder.getAbsolutePosition().getValueAsDouble() - offset.getRotations());
    }

    /**
     * Reset the module to the absoulte position
     */
    public void resetToAbs() {
        double corrected = encoder.getAbsolutePosition().getValueAsDouble() - offset.getRotations();
        turningMotor.setPosition(corrected);
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
                this.getRotation());
    }

    /**
     * Get the current position of the module
     * @return The position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                MathUtils.RPSToMPS(
                        driveMotor.getPosition().getValueAsDouble(),
                        SwerveConstants.WHEEL_CIRCUMFERENCE),
                this.getRotation());
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public void setVolts(double volts) {
        this.driveMotor.setVoltage(volts);
    }

    public void tick() {
        double turnMeasurement = this.getRotation().getRadians();
        double turnSetpoint = this.targetAngle.getRadians();

        double turnPidOut = -tProfiledPIDController.calculate(turnMeasurement, turnSetpoint);
        double turnFFOut = this.turnFF.calculate(tProfiledPIDController.getSetpoint().velocity);
        double turnVoltage = turnPidOut + turnFFOut;
        turningMotor.setVoltage(turnVoltage);

        double driveMeasurement =
                MathUtils.RPSToMPS(
                        this.driveMotor.getVelocity().getValueAsDouble(),
                        SwerveConstants.WHEEL_CIRCUMFERENCE);

        double drivePidOut = dPIDController.calculate(driveMeasurement, this.targetSpeedMps);
        double driveFFOut = driveFF.calculateWithVelocities(driveMeasurement, this.targetSpeedMps);
        double driveVoltage = drivePidOut + driveFFOut;
        driveMotor.setVoltage(driveVoltage);

        Logger.recordOutput("Swerve/Module " + moduleNumber + "/DrivePIDOut", drivePidOut);
        Logger.recordOutput("Swerve/Module " + moduleNumber + "/DriveVoltageOut", driveVoltage);
        // Logger.recordOutput("Swerve/Module " + moduleNumber + "/TurnVoltage", turnVoltage);

        Logger.recordOutput(
                "Swerve/Module " + moduleNumber + "/DriveReportedMPS", driveMeasurement);
        Logger.recordOutput(
                "Swerve/Module " + moduleNumber + "/DriveReportedRPS",
                this.driveMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput(
                "Swerve/Module " + moduleNumber + "/DriveTargetMPS", this.targetSpeedMps);

        Logger.recordOutput(
                "Swerve/Module " + moduleNumber + "/DrivePIDError", dPIDController.getError());
        Logger.recordOutput(
                "Swerve/Module " + moduleNumber + "/TurnPIDError",
                tProfiledPIDController.getPositionError());

        Logger.recordOutput("Swerve/Module " + moduleNumber + "/TurnPIDSetpoint", turnSetpoint);

        Logger.recordOutput("Swerve/Module " + moduleNumber + "/TurnMeasurement", turnMeasurement);
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
