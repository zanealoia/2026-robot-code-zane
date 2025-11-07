/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.teamdeadbolts.constants.SwerveConstants;

/**
 * Configuration for CTRE devices
 */
@SuppressWarnings("unchecked")
public class CtreConfigs {
    /* Confgis */
    /* Swerve */
    public static TalonFXConfiguration swerveTurningFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public static CANcoderConfiguration swerveCANcoderConfiguration = new CANcoderConfiguration();

    /* Trun motor tuning values */
    private static final LoggedNetworkNumber tCurrentLimit =
            new LoggedNetworkNumber("Tuning/Swerve/Turn/CurrentLimit", 0.0);

    /* Drive motor tuning values */
    private static final LoggedNetworkNumber dCurrentLimit =
            new LoggedNetworkNumber("Tuning/Swerve/Drive/CurrentLimit", 0.0);

    // private static final LoggedDashboardChooser dGearRatio =
    // new LoggedDashboardChooser<Integer>("Tuning/Swerve/Drive/GearRatio");

    static {
        init();
    }

    public static void init() {
        /* Turning motor config */
        /* Motor inverts and neutral modef */
        swerveTurningFXConfig.MotorOutput.Inverted = SwerveConstants.TURN_INVERTED_MODE;
        swerveTurningFXConfig.MotorOutput.NeutralMode = SwerveConstants.TURN_NEUTRAL_MODE;

        /* Gear ratios */
        swerveTurningFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.TURN_GEAR_RATIO;
        swerveTurningFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current limiting */
        swerveTurningFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveTurningFXConfig.CurrentLimits.SupplyCurrentLimit = tCurrentLimit.get();

        /* Drive Motor Config */
        swerveDriveFXConfig.MotorOutput.Inverted = SwerveConstants.DRIVE_INVERTED_MODE;
        swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveConstants.DRIVE_NEUTRAL_MODE;

        /* Gear ratio */
        // switch ((int) dGearRatio.get()) {
        //     case 0:
        //         swerveDriveFXConfig.Feedback.SensorToMechanismRatio =
        //                 SwerveConstants.DRIVE_GEAR_RATIO_R1;
        //         break;
        //     case 1:
        //         swerveDriveFXConfig.Feedback.SensorToMechanismRatio =
        //                 SwerveConstants.DRIVE_GEAR_RATIO_R2;
        //         break;
        //     case 2:
        //         swerveDriveFXConfig.Feedback.SensorToMechanismRatio =
        //                 SwerveConstants.DRIVE_GEAR_RATIO_R3;
        //         break;
        //     default:
        //         break;
        // }

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO_R1;

        /** Current limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = dCurrentLimit.get();

        /* Swerve CANCoder config */
        swerveCANcoderConfiguration.MagnetSensor.SensorDirection = SwerveConstants.SENSOR_DIRECTION;
    }
}
