package frc.robot.commands.Drive;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveSubsystem;

public class SetToCoast   extends CommandBase {
    private DriveSubsystem m_drive;

    public SetToCoast(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);
        
    }

    @Override
    public void execute(){
   
        m_drive.setToCoast();
    }
}
