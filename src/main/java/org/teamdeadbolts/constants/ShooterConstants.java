package org.teamdeadbolts.constants;

import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Pose3d;

public class ShooterConstants {
    public static final int SHOOTER_TURRET_MOTOR_CAN_ID = -1;
    public static final int SHOOTER_HOOD_MOTOR_CAN_ID = -1;
    public static final int SHOOTER_ABS_ENCODER_CAN_ID = -1;
    public static final int SHOOTER_WHEEL_MOTOR_CAN_ID = -1;

    public static final double SHOOTER_HOOD_MIN_ANGLE_DEGREES = 0.0;
    public static final double SHOOTER_HOOD_MAX_ANGLE_DEGREES = 45;

    public static final double TURRENT_MIN_POSITION_DEGREES = -270.0;
    public static final double TURRENT_MAX_POSITION_DEGREES = 270.0;

    public static final Pose3d PASS_LEFT_POSE_RED = new Pose3d();
    public static final Pose3d PASS_RIGHT_POSE_BLUE = new Pose3d();
    public static final Pose3d PASS_RIGHT_POSE_RED = new Pose3d();
    public static final Pose3d PASS_LEFT_POSE_BLUE = new Pose3d();
    public static final Pose3d SHOOT_POSE_RED = new Pose3d();
    public static final Pose3d SHOOT_POSE_BLUE = new Pose3d();

    public static final TalonFXConfiguration SHOOTER_TURRET_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration SHOOTER_HOOD_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration SHOOTER_WHEEL_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final CANcoderConfiguration SHOOTER_ABS_ENCODER_CONFIG = new CANcoderConfiguration();

    private static final SavedLoggedNetworkNumber shooterTurretMotorCurrentLimit = SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterTurretMotorCurrentLimit", 20);
    private static final SavedLoggedNetworkNumber shooterHoodMotorCurrentLimit = SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterHoodMotorCurrentLimit", 20);
    private static final SavedLoggedNetworkNumber shooterWheelMotorCurrentLimit = SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterWheelMotorCurrentLimit", 20);

    static {
        ConfigManager.getInstance().onReady(ShooterConstants::init);   
    }
    public static void init() {
        shooterTurretMotorCurrentLimit.initFromConfig();
        SHOOTER_TURRET_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_TURRET_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterTurretMotorCurrentLimit.get();

        shooterHoodMotorCurrentLimit.initFromConfig();
        SHOOTER_HOOD_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_HOOD_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterHoodMotorCurrentLimit.get();

        shooterWheelMotorCurrentLimit.initFromConfig();
        SHOOTER_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterWheelMotorCurrentLimit.get();
    }
}
