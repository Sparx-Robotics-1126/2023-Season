package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.AcquisitionSubsystem;

public class Elevate extends CommandBase {
private AcquisitionSubsystem m_acq;

    public Elevate(AcquisitionSubsystem acquisitionSubsystem) {
        m_acq = acquisitionSubsystem;
        addRequirements(acquisitionSubsystem);

    }

    public void initialize() {
        // m_acq.reset();
        // startAngle = m_drive.getHeading();
      }

      @Override
      public void execute() {
        m_acq.elevate();
  
          // m_drive.tankDrive(DriveConstants.kAutoDriveForwardSpeed,
          // DriveConstants.kAutoDriveForwardSpeed);
      }
  
      @Override
      public void end(boolean interrupted) {
          //m_drive.stop();
          
      }
  
   
}
