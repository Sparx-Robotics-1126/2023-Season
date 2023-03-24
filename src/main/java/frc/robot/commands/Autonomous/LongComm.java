package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;

public class LongComm extends SequentialCommandGroup {
   
    public LongComm(DriveSubsystem drives, AcquisitionSubsystem Acquisitions) {

        addCommands(new DriveDistanceCmd(drives, 4.7, .75));
    }
}
