package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.AcquisitionSubsystem;

public class ReturnToHome extends CommandBase {
    
   

    private AcquisitionSubsystem acquisition;

    public ReturnToHome(AcquisitionSubsystem acquisitionSubsystem) {
     
        acquisition = acquisitionSubsystem;
    }
   
    @Override
    public void execute() {
        if (!acquisition.getLowerLimitLeft()) {
            acquisition.setXMotorLeft(.05);
        } else {
            acquisition.setXMotorLeft(0); 
        }

        if(!acquisition.getLowerLimitRight()) {
            acquisition.setXMotorRight(.05);
        } else {
            acquisition.setXMotorRight(0); 
        }
        
        if(!acquisition.getUpperLimitLeft()) {
            acquisition.setYMotorLeft(.05);
        } else {
            acquisition.setYMotorLeft(0);
        }

        if(!acquisition.getUpperLimitRight()) {
            acquisition.setYMotorRight(.05);
        } else {
            acquisition.setYMotorRight(0);
        }
    }
    
    @Override
    public boolean isFinished() {
     return acquisition.getLowerLimitLeft() && acquisition.getLowerLimitRight() && acquisition.getUpperLimitLeft() && acquisition.getUpperLimitRight();
    }
}
