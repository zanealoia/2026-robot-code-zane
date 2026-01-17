package org.teamdeadbolts.subsystems;

import java.io.ObjectInputFilter.Config;

import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.HopperConstants;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    public enum State {
        HOLD,
        FAST_UP,
        FAST_DOWN,
        SLOW_UP,
        SLOW_DOWN,
    }

    private State targetState = State.HOLD;

    private TalonFX hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_CAN_ID);

    private SavedLoggedNetworkNumber hopperMotorFastVolts = SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorFastVolts", 1.0);
    private SavedLoggedNetworkNumber hopperMotorSlowVolts = SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorSlowVolts", 0.5);
    
    public HopperSubsystem() {
        ConfigManager.getInstance().onReady(() -> {
            HopperConstants.init();
            hopperMotor.getConfigurator().apply(HopperConstants.HOPPER_MOTOR_CONFIG);
        });
    }

    public void setState(State newState) {
        targetState = newState;
    }

    public State getState() {
        return targetState;
    }

    @Override
    public void periodic() {
        switch (targetState) {
            case FAST_DOWN:
                hopperMotor.setVoltage(-hopperMotorFastVolts.get());
                break;
            case FAST_UP:
                hopperMotor.setVoltage(hopperMotorFastVolts.get());
                break;
            case HOLD:
                hopperMotor.setVoltage(0);
                break;
            case SLOW_DOWN:
                hopperMotor.setVoltage(-hopperMotorSlowVolts.get());
                break;
            case SLOW_UP:
                hopperMotor.setVoltage(hopperMotorSlowVolts.get());
                break;
        }

        Logger.recordOutput("Hopper/TargetState", targetState);

    }
}
