package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.drives.DrivesCommand;
import frc.robot.subsystem.DriveSubsystem;

public class DriveForwardTime extends SequentialCommandGroup{
    public DriveForwardTime(DriveSubsystem driveSubsystem, int driveSeconds){
        super(new DriveTimed(driveSubsystem).withTimeout(driveSeconds));
    }
}