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
        if (acquisition.getUpperLimit())
            acquisition.setXMotor(0);
        else
            acquisition.setXMotor(-1);
        
        if (acquisition.getLowerLimitLeft())
            acquisition.setYMotorLeft(-1);
        else
            acquisition.setYMotorLeft(0);
        
        if (acquisition.getLowerLimitRight())
            acquisition.setYMotorRight(-1);
        else
            acquisition.setYMotorRight(0);
    }
    
    @Override
    public boolean isFinished() {
        if (acquisition.getLowerLimitLeft() && acquisition.getLowerLimitRight() && acquisition.getUpperLimit()) {
        acquisition.reset();
        return true;
        }

        return false;
    }
}
