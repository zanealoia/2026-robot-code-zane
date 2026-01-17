package org.teamdeadbolts.constants;

import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class HopperConstants {
    public static final int HOPPER_MOTOR_CAN_ID = -1;

    public static final TalonFXConfiguration HOPPER_MOTOR_CONFIG = new TalonFXConfiguration();

    static {
        ConfigManager.getInstance().onReady(HopperConstants::init);
    }

    private static final SavedLoggedNetworkNumber HopperMotorCurrentLimit = SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorCurrentLimit", 20);
    public static void init() {
        HopperMotorCurrentLimit.initFromConfig();
        HOPPER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        HOPPER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = HopperMotorCurrentLimit.get();
    }
}
