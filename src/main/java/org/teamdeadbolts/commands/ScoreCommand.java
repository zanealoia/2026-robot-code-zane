package org.teamdeadbolts.commands;

import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private HopperSubsystem hopperSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public ScoreCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, HopperSubsystem hopperSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(shooterSubsystem, indexerSubsystem, hopperSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setState(ShooterSubsystem.State.SHOOT);
        indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
        hopperSubsystem.setState(HopperSubsystem.State.SLOW_DOWN);
        intakeSubsystem.setState(IntakeSubsystem.State.SHOOT);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setState(ShooterSubsystem.State.OFF);
        indexerSubsystem.setState(IndexerSubsystem.State.OFF);
        hopperSubsystem.setState(HopperSubsystem.State.HOLD);
        intakeSubsystem.setState(IntakeSubsystem.State.STOWED);
    }

    @Override
    public boolean isFinished() {
        return false; // TODO: Check active hub and if balls
    }
}
