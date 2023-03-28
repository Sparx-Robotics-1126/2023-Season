package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Acquisition.MoveTo;
import frc.robot.commands.Drive.DriveToPitch;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import static  frc.robot.Constants.FieldConstants.*;

public class ReverseLongBalance extends SequentialCommandGroup{
    public ReverseLongBalance(DriveSubsystem drive, AcquisitionSubsystem acquisition) {

        addCommands(new MoveTo( HIGH_CUBE_X, HIGH_CUBE_Y, acquisition));
        addCommands(new WaitCommand(3));
        addCommands(new InstantCommand(() -> acquisition.grabberOpen()));
        addCommands(new MoveTo(0, 0, acquisition));
         addCommands(new WaitCommand(1));
        addCommands(new InstantCommand(() -> acquisition.grabberClose()));
        // addCommands(new MoveTo(0, 0, acquisition));


        addCommands(new DriveToPitch(drive, .65, -10,true ,true));
        addCommands(new WaitCommand(1));
    //     // addCommands(new DriveDistance(drive, -.3, .5));
       addCommands(new BalanceOnChargeStation(drive).withTimeout(8));
        
    }
}
