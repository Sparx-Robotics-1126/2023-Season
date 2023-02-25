package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.DriveSubsystem;

public class DriveMeasurements extends SequentialCommandGroup{


    public DriveMeasurements(DriveSubsystem drive) {

        addCommands(new DriveDistance(drive, 1, .5));
    }
    
}
