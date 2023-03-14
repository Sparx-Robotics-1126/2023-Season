package frc.robot.commands;

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
    public void execute() 
    {
        if (acquisition.getXLimit())
            acquisition.setXMotor(0);
        else
            acquisition.setXMotor(-1);
        
        if (acquisition.getYLimit())
            acquisition.setYMotor(0);
        else
            acquisition.setYMotor(-1);
    }

    @Override
    public void end(boolean interrupted)
    {
        if (!interrupted)
            acquisition.reset();
    }
    
    @Override
    public boolean isFinished() 
    {
        return acquisition.getXLimit() && acquisition.getYLimit();
    }
}
