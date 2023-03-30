package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Drive.DriveToPitch;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;

import frc.robot.commands.Acquisition.MoveTo;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import static  frc.robot.Constants.FieldConstants.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class ReverseShortBalance extends SequentialCommandGroup{
    public ReverseShortBalance(DriveSubsystem drive, AcquisitionSubsystem acquisition) {
        
        addCommands(new MoveTo( MID_CUBE_X, MID_CUBE_Y, acquisition));
        addCommands(new WaitCommand(3));
        addCommands(new InstantCommand(() -> acquisition.grabberOpen()));
        addCommands(new MoveTo(0, 0, acquisition));
         addCommands(new WaitCommand(1));
        addCommands(new InstantCommand(() -> acquisition.grabberClose()));
        addCommands(new MoveTo(0, 0, acquisition));


        addCommands(new DriveToPitch(drive, .65, -10,true ,true));
        addCommands(new WaitCommand(.5));
    //     // addCommands(new DriveDistance(drive, -.3, .5));
       addCommands(new BalanceOnChargeStation(drive).withTimeout(8));
    }
    
}
