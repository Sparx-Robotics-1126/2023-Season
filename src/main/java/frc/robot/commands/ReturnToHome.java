package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.Constants.AcquisitionConstants;

public class ReturnToHome extends CommandBase {
    
   

    private AcquisitionSubsystem acquisition;

    public ReturnToHome(AcquisitionSubsystem acquisitionSubsystem) {
     
        acquisition = acquisitionSubsystem;
    }
   
    @Override
    public void execute() {
        if (!acquisition.getLowerLimitLeft()) {
            acquisition.setXMotorLeft(-1);
        } else {
            acquisition.setXMotorLeft(0); 
        }

        if(!acquisition.getLowerLimitRight()) {
            acquisition.setXMotorRight(-1);
        } else {
            acquisition.setXMotorRight(0); 
        }
        
        if(!acquisition.getUpperLimitLeft()) {
            acquisition.setYMotorLeft(-1);
        } else {
            acquisition.setYMotorLeft(0);
        }

    }
    
    @Override
    public boolean isFinished() {
     if (acquisition.getLowerLimitLeft() && acquisition.getLowerLimitRight() && acquisition.getUpperLimitLeft() && acquisition.getUpperLimitRight()) {
        acquisition.reset();
        return true;
     }
     return false;
    }
}
