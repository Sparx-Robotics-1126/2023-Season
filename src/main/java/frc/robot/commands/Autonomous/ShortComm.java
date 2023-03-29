package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Acquisition.MoveTo;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import static  frc.robot.Constants.FieldConstants.*;

public class ShortComm extends SequentialCommandGroup {
   
    public ShortComm(DriveSubsystem drive, AcquisitionSubsystem acquisition) {
        addCommands(new MoveTo( MID_CUBE_X, MID_CUBE_Y, acquisition));
        addCommands(new WaitCommand(3));
        addCommands(new InstantCommand(() -> acquisition.grabberOpen()));
        addCommands(new MoveTo(0, 0, acquisition));
         addCommands(new WaitCommand(2));
        addCommands(new InstantCommand(() -> acquisition.grabberClose()));
        addCommands(new DriveDistanceCmd(drive, -2, .75));
    }
}
