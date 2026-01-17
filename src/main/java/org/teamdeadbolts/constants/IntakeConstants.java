package org.teamdeadbolts.constants;

import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeConstants {
    public static final int INTAKE_ARM_MOTOR_CAN_ID = -1;
    public static final int INTAKE_WHEEL_MOTOR_CAN_ID = -1;
    public static final int INTAKE_ABS_ENCODER_CAN_ID = -1;

    public static final TalonFXConfiguration INTAKE_ARM_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration INTAKE_WHEEL_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final CANcoderConfiguration INTAKE_ABS_ENCODER_CONFIG = new CANcoderConfiguration();

    private static final SavedLoggedNetworkNumber intakeArmMotorCurrentLimit = SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeArmMotorCurrentLimit", 20);
    private static final SavedLoggedNetworkNumber intakeWheelMotorCurrentLimit = SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeWheelMotorCurrentLimit", 20);

    static {
        ConfigManager.getInstance().onReady(IntakeConstants::init);
    }

    public static void init() {
        intakeArmMotorCurrentLimit.initFromConfig();
        INTAKE_ARM_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        INTAKE_ARM_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = intakeArmMotorCurrentLimit.get();

        intakeWheelMotorCurrentLimit.initFromConfig();
        INTAKE_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        INTAKE_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = intakeWheelMotorCurrentLimit.get();
    }
}
