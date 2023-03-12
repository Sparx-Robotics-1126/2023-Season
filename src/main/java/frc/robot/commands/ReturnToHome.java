package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.AcquisitionSubsystem;

public class ReturnToHome extends CommandBase 
{
    private AcquisitionSubsystem acquisition;

    public ReturnToHome(AcquisitionSubsystem acquisitionSubsystem) {
     
        acquisition = acquisitionSubsystem;
    }
   
    @Override
    public void execute() {
        if (acquisition.getXLimit())
            acquisition.setXMotor(0);
        else
            acquisition.setXMotor(-1);
        
        /*if (acquisition.getYLimitLeft())
            acquisition.setYMotorLeft(-1);
        else
            acquisition.setYMotorLeft(0);
        
        if (acquisition.getYLimitRight())
            acquisition.setYMotorRight(-1);
        else
            acquisition.setYMotorRight(0);*/
    }
    
    @Override
    public boolean isFinished() {
        if (acquisition.getXLimit())// && acquisition.getYLimitRight() && acquisition.getYLimitLeft()) 
        {
            acquisition.reset();
            return true;
        }

        return false;
    }
}
