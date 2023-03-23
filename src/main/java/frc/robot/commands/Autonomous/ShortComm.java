package frc.robot.commands.Autonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;

public class ShortComm extends SequentialCommandGroup {
   
    public ShortComm(DriveSubsystem drive, AcquisitionSubsystem acquisitions) {
        
        addCommands(new DriveDistanceCmd(drive, 3.31, .75));
    }
}
