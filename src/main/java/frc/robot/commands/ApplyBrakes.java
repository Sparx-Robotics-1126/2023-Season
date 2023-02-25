package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveSubsystem;

public class ApplyBrakes  extends CommandBase  {
    private DriveSubsystem m_drive;

    public ApplyBrakes(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);
        
    }

    @Override
    public void execute(){
        m_drive.applyBrakes();
    }
    
}
