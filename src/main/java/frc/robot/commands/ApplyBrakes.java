package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveSubsystem;

public class ApplyBrakes extends CommandBase {
    private DriveSubsystem m_drive;
    private Timer m_timer;
    private double m_availAfter;

    public ApplyBrakes(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);

    }

    public ApplyBrakes(DriveSubsystem drive, Timer timer, double availAfter) {

        m_drive = drive;
        m_timer = timer;
        m_availAfter = availAfter;
        addRequirements(drive);
    }

    @Override
    public void execute(){
        m_drive.m_driveDifferential.feed();
       
        
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > m_availAfter) {
            return m_drive.applyBrakes();
 
         }
         return true;
    }
   
}
