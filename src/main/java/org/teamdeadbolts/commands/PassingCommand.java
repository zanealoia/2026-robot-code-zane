/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.subsystems.ShooterSubsystem;

public class PassingCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public enum Target {
        LEFT,
        RIGHT;
    }

    private Target target;

    public PassingCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            IntakeSubsystem intakeSubsystem,
            Target target) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.target = target;

        addRequirements(shooterSubsystem, indexerSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (target == Target.LEFT) {
            shooterSubsystem.setState(ShooterSubsystem.State.PASS_LEFT);
        } else {
            shooterSubsystem.setState(ShooterSubsystem.State.PASS_RIGHT);
        }

        indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
        intakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setState(ShooterSubsystem.State.OFF);
        indexerSubsystem.setState(IndexerSubsystem.State.OFF);
        intakeSubsystem.setState(IntakeSubsystem.State.STOWED);
    }
}
