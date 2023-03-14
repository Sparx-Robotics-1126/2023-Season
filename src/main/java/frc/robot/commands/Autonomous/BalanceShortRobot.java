package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.DriveToPitch;
import frc.robot.subsystem.DriveSubsystem;

public class BalanceShortRobot extends SequentialCommandGroup {
 
  public BalanceShortRobot(DriveSubsystem drive) {

    // Good short balance group
     addCommands(new DriveToPitch(drive, .65, 10,false,false ));
     addCommands(new WaitCommand(1));
     addCommands(new DriveToPitch(drive, .4, -2, false, false));
     addCommands(new WaitCommand(1));
     addCommands(new DriveToPitch(drive, .3, 0, true, false));

   
  }
}

