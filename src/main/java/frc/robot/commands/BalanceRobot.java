package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.DriveSubsystem;

public class BalanceRobot extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public BalanceRobot(DriveSubsystem drive) {

    // Good short balance group
    // addCommands(new DriveToPitch(drive, .65, 10,false,false));
    // addCommands(new WaitCommand(1));
    // addCommands(new DriveToPitch(drive, .4, -2, false, false));
    // addCommands(new WaitCommand(1));
    // addCommands(new DriveToPitch(drive, .3, 0, true, false));

    // moves over docking station
    // drive to ramp and up a little
    addCommands(new DriveToPitch(drive, .65, 10, false, false, "Step 1"));
    // addCommands(new WaitCommand(1));
    // drive until ramp startsg going down
    
    addCommands(new DriveToPitch(drive, .5, -2, false, false, "Step 2"));
    // addCommands(new WaitCommand(1));
    // drive forward until floor
    // addCommands(new DriveToPitch(drive, .4, 0, false, false));
    // addCommands(new WaitCommand(1));

    // // //start to backup onto station
    // //drive forward to make space
    // SmartDashboard.putString("CURRENT_STEP", "Step 3");
    addCommands(new DriveDistance(drive, 2, .4));
    addCommands(new WaitCommand(1));
    // drive backwards for distance hopefully on ramp
    addCommands(new DriveToPitch(drive, .65, -10, true, true, "Step 4"));
    // addCommands(new DriveDistance(drive, .65, -.65));
    addCommands(new WaitCommand(1));
    // //drive until pitch is 2 backwards
    addCommands(new DriveToPitch(drive, .4, 2, true, false, "Step 5"));
    addCommands(new WaitCommand(1));
    // addCommands(new DriveToPitch(drive, .35, -2, true));
    // addCommands(new WaitCommand(1));
    addCommands(new DriveToPitch(drive, .4,-2,  false, false, "Step 6"));
    addCommands(new WaitCommand(1));
    // drive until ramp startsg going down
    addCommands(new DriveToPitch(drive, .3, 0, true, false, "Step 7"));
  }
}

// The code you typed in **RobotPreferences.java** should look like this

// '''java
// public static double autoDelay() {
// return Preferences.getInstance().getDouble("autoDelay", 5.0);
// }

// public static double autoDriveDistance() {
// return Preferences.getInstance().getDouble("autoDriveDistance", 12.0);
// }