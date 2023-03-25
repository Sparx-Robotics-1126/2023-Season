package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.commands.Drive.DriveDistanceCmd;

public class ReverseShortComm extends SequentialCommandGroup
{
    public ReverseShortComm(DriveSubsystem drive, AcquisitionSubsystem Acquisition) {
       
        addCommands(new DriveDistanceCmd(drive, -2.5, -.75));
    }
}
