package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.subsystem.DriveSubsystem;

public class AutoDistances  extends SequentialCommandGroup {
    public AutoDistances( DriveSubsystem drive) {
       
        addCommands(new DriveDistance(drive, 2, .5));
        addCommands(new WaitCommand(2));
        addCommands(new DriveDistance(drive, -1, .5));
        addCommands(new WaitCommand(2));
        addCommands(new DriveDistance(drive, 1, .5));
        addCommands(new WaitCommand(2));
        addCommands(new InstantCommand(() -> drive.applyBrakes()));
      }
}
