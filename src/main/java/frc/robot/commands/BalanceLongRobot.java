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
    addCommands(new DriveToPitch(drive, .5, -2, false, false));
   
    addCommands(new DriveDistance(drive, 2, .65));
     addCommands(new WaitCommand(.5));
    // drive backwards for distance hopefully on ramp
    addCommands(new DriveToPitch(drive, .65, -10, true, true));
    // // addCommands(new DriveDistance(drive, .65, -.65));
    //  addCommands(new WaitCommand(1));
    // // //drive until pitch is 2 backwards
    addCommands(new DriveToPitch(drive, .3, 2, true, false));
    // addCommands(new WaitCommand(1));
    // // addCommands(new DriveToPitch(drive, .35, -2, true));
   // addCommands(new WaitCommand(1));
    addCommands(new DriveToPitch(drive, .2, 0, false, false));
   
    // addCommands(new DriveToPitch(drive, .4, -2, false, false, "Step 6"));
    // addCommands(new WaitCommand(1));
    // // drive until ramp startsg going down
    // addCommands(new DriveToPitch(drive, .3, 0, true, false, "Step 7"));
  }
}

