/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import org.teamdeadbolts.subsystems.drive.SwerveModule.SwerveModuleConfig;

public class SwerveConstants {
    public static final double WHEEL_CIRCUMFERENCE = 0.0;
    public static final double CHASSIS_SIZE = Units.inchesToMeters(30);
    public static final SensorDirectionValue SENSOR_DIRECTION =
            SensorDirectionValue.Clockwise_Positive;
    /* Turning constants */
    public static final InvertedValue TURN_INVERTED_MODE = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue TURN_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double TURN_GEAR_RATIO = 0.0;

    /* Driving constants */
    public static final InvertedValue DRIVE_INVERTED_MODE = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double DRIVE_GEAR_RATIO = 0.0;

    /* Module constants */
    public static final SwerveModuleConfig FRONT_RIGHT_CONFIG =
            new SwerveModuleConfig(0, new Rotation2d(), 0, 0, 0);
    public static final SwerveModuleConfig FRONT_LEFT_CONFIG =
            new SwerveModuleConfig(1, new Rotation2d(), 0, 0, 0);
    public static final SwerveModuleConfig BACK_RIGHT_CONFIG =
            new SwerveModuleConfig(2, new Rotation2d(), 0, 0, 0);
    public static final SwerveModuleConfig BACK_LEFT_CONFIG =
            new SwerveModuleConfig(3, new Rotation2d(), 0, 0, 0);

    /* Kinematics */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(
                    new Translation2d(CHASSIS_SIZE / 2.0, CHASSIS_SIZE / 2.0),
                    new Translation2d(CHASSIS_SIZE / 2.0, -CHASSIS_SIZE / 2.0),
                    new Translation2d(-CHASSIS_SIZE / 2.0, CHASSIS_SIZE / 2.0),
                    new Translation2d(-CHASSIS_SIZE / 2.0, -CHASSIS_SIZE / 2.0));
}
