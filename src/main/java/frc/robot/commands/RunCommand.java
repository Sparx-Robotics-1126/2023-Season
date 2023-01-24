package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystem.Drives;

public class RunCommand extends CommandBase {
    private final DifferentialDrive m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    public RunCommand(DifferentialDrive subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
        m_drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(m_drive.drive);
    }

    @Override
  public void execute() {
    m_drive.tankDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
  }
}
