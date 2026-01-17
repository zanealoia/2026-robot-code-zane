package org.teamdeadbolts.subsystems;

import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public enum State {
        OFF,
        SPINUP,
        SHOOT,
        PASS_LEFT,
        PASS_RIGHT;
    }

    private State targetState = State.OFF;

    private TalonFX turretMotor = new TalonFX(ShooterConstants.SHOOTER_TURRET_MOTOR_CAN_ID);
    private TalonFX hoodMotor = new TalonFX(ShooterConstants.SHOOTER_HOOD_MOTOR_CAN_ID);
    private TalonFX wheelMotor = new TalonFX(ShooterConstants.SHOOTER_WHEEL_MOTOR_CAN_ID);

    private CANcoder absEncoder = new CANcoder(ShooterConstants.SHOOTER_ABS_ENCODER_CAN_ID);

    private PIDController hoodController = new PIDController(0.0, 0.0, 0.0);
    private PIDController turretController = new PIDController(0.0, 0.0, 0.0);
    private PIDController wheelController = new PIDController(0.0, 0.0, 0.0);

    private SavedLoggedNetworkNumber hoodControllerP = SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kP", 0.1);
    private SavedLoggedNetworkNumber hoodControllerI = SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kI", 0.0);
    private SavedLoggedNetworkNumber hoodControllerD = SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kD", 0.0);

    private SavedLoggedNetworkNumber turretControllerP = SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kP", 0.1);
    private SavedLoggedNetworkNumber turretControllerI = SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kI", 0.0);
    private SavedLoggedNetworkNumber turretControllerD = SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kD", 0.0);

    private SavedLoggedNetworkNumber wheelControllerP = SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kP", 0.1);
    private SavedLoggedNetworkNumber wheelControllerI = SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kI", 0.0);
    private SavedLoggedNetworkNumber wheelControllerD = SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kD", 0.0);


    private SavedLoggedNetworkNumber shooterWheelSpinupSpeed = SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterWheelSpinupSpeed", 5000.0); // RPM


    public ShooterSubsystem() {
        ConfigManager.getInstance().onReady(this::reconfigure);
    }

    private void reconfigure() {
        hoodController.setPID(hoodControllerP.get(), hoodControllerI.get(), hoodControllerD.get());
        turretController.setPID(turretControllerP.get(), turretControllerI.get(), turretControllerD.get());
        wheelController.setPID(wheelControllerP.get(), wheelControllerI.get(), wheelControllerD.get());

        ShooterConstants.init();
        hoodMotor.getConfigurator().apply(ShooterConstants.SHOOTER_HOOD_MOTOR_CONFIG);
        turretMotor.getConfigurator().apply(ShooterConstants.SHOOTER_TURRET_MOTOR_CONFIG);
        wheelMotor.getConfigurator().apply(ShooterConstants.SHOOTER_WHEEL_MOTOR_CONFIG);
        absEncoder.getConfigurator().apply(ShooterConstants.SHOOTER_ABS_ENCODER_CONFIG);
    }

    public void setState(State newState) {
        targetState = newState;
    }

    public State getState() {
        return targetState;
    }

    @Override
    public void periodic() {
        double currentHoodAngle = absEncoder.getPosition().getValueAsDouble();
        double targetHoodAngle = currentHoodAngle;

        double currentTurrentPosition = turretMotor.getPosition().getValueAsDouble();
        double targetTurretPosition = currentTurrentPosition;

        double currentWheelSpeed = wheelMotor.getVelocity().getValueAsDouble();
        double targetWheelSpeed = currentWheelSpeed;

        Pose2d robotPose = RobotState.getInstance().getRobotPose().toPose2d();
        ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeRobotVelocities();


        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        switch (targetState) {
            case OFF:
                wheelMotor.setVoltage(0);
                targetHoodAngle = ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES;
                break;
            case PASS_LEFT:
            case PASS_RIGHT:
                Pose3d targetPose;

                if (alliance == Alliance.Red) {
                    targetPose = targetState == State.PASS_LEFT ? ShooterConstants.PASS_LEFT_POSE_RED : ShooterConstants.PASS_RIGHT_POSE_RED;
                } else {
                    targetPose = targetState == State.PASS_LEFT ? ShooterConstants.PASS_LEFT_POSE_BLUE : ShooterConstants.PASS_RIGHT_POSE_BLUE;
                }

                ShooterAimingParameters aimingParams = calculateAimingParameters(targetPose, robotPose, robotSpeeds, true);
                targetHoodAngle = aimingParams.hoodAngle;
                targetTurretPosition = aimingParams.turrentAngle;
                targetWheelSpeed = aimingParams.wheelSpeed;
                break;
            case SHOOT:
                Pose3d shootTargetPose = (alliance == Alliance.Red) ? ShooterConstants.SHOOT_POSE_RED : ShooterConstants.SHOOT_POSE_BLUE;
                ShooterAimingParameters shootAimingParams = calculateAimingParameters(shootTargetPose, robotPose, robotSpeeds, false);
                targetHoodAngle = shootAimingParams.hoodAngle;
                targetTurretPosition = shootAimingParams.turrentAngle;
                targetWheelSpeed = shootAimingParams.wheelSpeed;
                break;
            case SPINUP:
                targetWheelSpeed = shooterWheelSpinupSpeed.get();
                targetHoodAngle = ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES;
                break;
        }

        double hoodOutput = hoodController.calculate(currentHoodAngle, targetHoodAngle);
        hoodMotor.setVoltage(hoodOutput);

        double wheelOutput = wheelController.calculate(currentWheelSpeed, targetWheelSpeed);
        wheelMotor.setVoltage(wheelOutput);

        targetTurretPosition = calculateTurrentSetpoint(currentTurrentPosition, Math.toRadians(targetTurretPosition));
        double turretOutput = turretController.calculate(currentTurrentPosition, targetTurretPosition);
        turretMotor.setVoltage(turretOutput);

        Logger.recordOutput("Shooter/TargetState", targetState);
        Logger.recordOutput("Shooter/TargetHoodAngle", targetHoodAngle);
        Logger.recordOutput("Shooter/CurrentHoodAngle", currentHoodAngle);
        Logger.recordOutput("Shooter/TargetTurretPosition", targetTurretPosition);
        Logger.recordOutput("Shooter/CurrentTurretPosition", currentTurrentPosition);
        Logger.recordOutput("Shooter/TargetWheelSpeed", targetWheelSpeed);
        Logger.recordOutput("Shooter/CurrentWheelSpeed", currentWheelSpeed);


    }

    private double calculateTurrentSetpoint(double currentAngle, double targetAngle) {
        double error = ((targetAngle - currentAngle + Math.PI) % (2 * Math.PI)) - Math.PI;
        
        if (error < 0) error += 2 * Math.PI;
        error -= Math.PI;

        double shortestPath = currentAngle + error;

        if (shortestPath > Math.toRadians(ShooterConstants.TURRENT_MAX_POSITION_DEGREES)) {
            return shortestPath - 2 * Math.PI;
        } else if (shortestPath < Math.toRadians(ShooterConstants.TURRENT_MIN_POSITION_DEGREES)) {
            return shortestPath + 2 * Math.PI;
        }
        return shortestPath;
    }

    private ShooterAimingParameters calculateAimingParameters(Pose3d target, Pose2d current, ChassisSpeeds robotSpeedsFieldRelative, boolean passing) {
        throw new UnsupportedOperationException("Shooter aiming calculation not yet implemented");
    }

    private record ShooterAimingParameters(double hoodAngle, double turrentAngle, double wheelSpeed) {}

}