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
            acquisition.setXMotorLeft(-AcquisitionConstants.MOTOR_SPEED);
        } else {
            acquisition.setXMotorLeft(0); 
        }

        if(!acquisition.getLowerLimitRight()) {
            acquisition.setXMotorRight(-AcquisitionConstants.MOTOR_SPEED);
        } else {
            acquisition.setXMotorRight(0); 
        }
        
        if(!acquisition.getUpperLimitLeft()) {
            acquisition.setYMotorLeft(-AcquisitionConstants.MOTOR_SPEED);
        } else {
            acquisition.setYMotorLeft(0);
        }

        if(!acquisition.getUpperLimitRight()) {
            acquisition.setYMotorRight(-AcquisitionConstants.MOTOR_SPEED);
        } else {
            acquisition.setYMotorRight(0);
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
