package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.DriveSubsystem;

public class BalanceLongRobot extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public BalanceLongRobot(DriveSubsystem drive) {
   
    // moves over docking station
    // drive to ramp and up a little
    addCommands(new DriveToPitch(drive, .65, 10, false, false));
    // drive until ramp startsg going down
    //addCommands(new DriveToPitch(drive, .5, -2, false, false));
    addCommands(new DriveDistance(drive, 3, .7));
    addCommands(new WaitCommand(.5));
    // drive backwards for distance hopefully on ramp
    addCommands(new DriveToPitch(drive, .65, -10, true, true));
    // // //drive until pitch is 2 backwards
    addCommands(new DriveToPitch(drive, .35, -1, true, false));
    //drive forward and balance to 0.
    //addCommands(new DriveToPitch(drive, .28, 0, false, false));
  }
}

