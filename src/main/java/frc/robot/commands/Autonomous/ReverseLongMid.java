package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AcquisitionConstants;
import frc.robot.commands.Acquisition.MoveTo;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Drive.DriveToPitch;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import static  frc.robot.Constants.FieldConstants.*;


public class ReverseLongMid extends SequentialCommandGroup{
    public ReverseLongMid(DriveSubsystem drive, AcquisitionSubsystem acquisition) {
        
        //Places Cube in Mid Node
        addCommands(new MoveTo( MID_CUBE_X, MID_CUBE_Y, acquisition));
        addCommands(new WaitCommand(3));
        addCommands(new InstantCommand(() -> acquisition.grabberOpen()));
        addCommands(new MoveTo(0, 0, acquisition));
        addCommands(new WaitCommand(1));
        addCommands(new InstantCommand(() -> acquisition.grabberClose()));

        //Drives Backwards over charge station, balances
        addCommands(new DriveToPitch(drive, 0.5, -10, true, true));
    // // drive until ramp starts going down
        addCommands(new DriveDistance(drive, -1.251, .5));
        addCommands(new WaitCommand(.5));
    // drive backwards for distance hopefully on ramp
        addCommands(new DriveToPitch(drive, .65, 10, false, false));
        addCommands(new WaitCommand(.5));
        addCommands(new DriveDistance(drive, .35, .5));
        addCommands(new BalanceOnChargeStation(drive));

    
    }
}
