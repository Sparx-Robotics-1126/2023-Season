package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.DriveToPitch;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;

public class ReverseShortBalance extends SequentialCommandGroup{
    public ReverseShortBalance(DriveSubsystem drive, AcquisitionSubsystem Acquisition) {
        

        addCommands(new DriveToPitch(drive, .65, -10,true ,true));
        addCommands(new WaitCommand(1));
    
       addCommands(new BalanceOnChargeStation(drive).withTimeout(5));
    }
    
}
