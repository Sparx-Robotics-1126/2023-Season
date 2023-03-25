package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;

public class ReverseLongComm extends SequentialCommandGroup {
    public ReverseLongComm(DriveSubsystem drives, AcquisitionSubsystem acquisition) {
        
        addCommands(new DriveDistanceCmd(drives, -8.83, -.75));
    }
}
