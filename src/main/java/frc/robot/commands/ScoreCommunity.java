package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.DriveSubsystem;

public class ScoreCommunity extends SequentialCommandGroup {
    
    public ScoreCommunity(DriveSubsystem drive) {

        //moving forward to score cone
        addCommands(new DriveDistance(drive, .5, .7).withTimeout(5));
        addCommands(new WaitCommand(1));
//moving backward to leave commmunity
        addCommands(new DriveDistance(drive, -3, .7).withTimeout(10));

    }
}


