package org.teamdeadbolts.commands;

import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private HopperSubsystem hopperSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, HopperSubsystem hopperSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.hopperSubsystem = hopperSubsystem;

        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
        indexerSubsystem.setState(IndexerSubsystem.State.INTAKE);
        hopperSubsystem.setState(HopperSubsystem.State.FAST_UP);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setState(IntakeSubsystem.State.DEPLOYED);
        indexerSubsystem.setState(IndexerSubsystem.State.OFF);
        hopperSubsystem.setState(HopperSubsystem.State.HOLD);
    }
}
