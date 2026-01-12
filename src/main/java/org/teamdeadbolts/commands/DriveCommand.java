/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;

public class DriveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier sidewaysSupplier;
    private DoubleSupplier rotationSupplier;
    private boolean fieldRelative;

    /**
     * Command to drive swerve
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param forwardSupplier A supplier for forward motion (in <strong>m/s</strong>)
     * @param sidewaysSupplier A supplier for sideways motion (in <strong>m/s</strong>)
     * @param rotationSuplier A supplier for rotaional motion (in <strong>rads/s</strong>)
     * @param fieldRelative Weather or not to drive the robot field relative
     */
    public DriveCommand(
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier forwardSupplier,
            DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationSuplier,
            boolean fieldRelative) {
        this.swerveSubsystem = swerveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.sidewaysSupplier = sidewaysSupplier;
        this.rotationSupplier = rotationSuplier;
        this.fieldRelative = fieldRelative;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Logger.recordOutput("ForwardMS", forwardSupplier.getAsDouble());
        Logger.recordOutput("SidewaysMS", sidewaysSupplier.getAsDouble());
        Logger.recordOutput("Angle RadsS", rotationSupplier.getAsDouble());

        swerveSubsystem.drive(
                new Translation2d(forwardSupplier.getAsDouble(), sidewaysSupplier.getAsDouble()),
                rotationSupplier.getAsDouble(),
                fieldRelative,
                false,
                false);
    }
}
