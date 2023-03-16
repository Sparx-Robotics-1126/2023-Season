package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.commands.Acquisition.MoveTo;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.subsystem.AcquisitionSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ScoreCommunity extends SequentialCommandGroup {
    
    public ScoreCommunity(DriveSubsystem drive, AcquisitionSubsystem acquisitions) {  
    
    //moving forward to score cone
    addCommands(new DriveDistance(drive, .2, .7).withTimeout(5));
    addCommands(new MoveTo(0, .2, acquisitions));
    addCommands(new InstantCommand(() -> acquisitions.grabberOpen()));
    
    //addCommands(new WaitCommand(1));
    //release cone
    addCommands((new InstantCommand( () -> acquisitions.grabberOpen())));
    addCommands(new WaitCommand(.5));
    //moving backward to leave commmunity
    addCommands(new DriveDistance(drive, -3, .7).withTimeout(10));

    }
}


