package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.DriveSubsystem;

public class DriveForwardTime extends SequentialCommandGroup{
    public DriveForwardTime(DriveSubsystem driveSubsystem, int driveSeconds){
        super(new DriveTimed(driveSubsystem).withTimeout(driveSeconds));
    }
}