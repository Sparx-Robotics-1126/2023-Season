package frc.robot.commands.Acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.AcquisitionSubsystem;

public class ReturnToHome extends CommandBase 
{
    private AcquisitionSubsystem acquisition;

    public ReturnToHome(AcquisitionSubsystem acquisitionSubsystem) 
    { 
        acquisition = acquisitionSubsystem;
    }
   
    @Override
    public void initialize() 
    {
        acquisition.xMoveTo(0);
        acquisition.yMoveTo(0);
    }
    
    @Override
    public boolean isFinished() 
    {
        return acquisition.atSetpoint();
    }
}
