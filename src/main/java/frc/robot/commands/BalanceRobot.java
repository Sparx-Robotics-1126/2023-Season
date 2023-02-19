package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.DriveSubsystem;

public class BalanceRobot extends SequentialCommandGroup {
  /**
  * Add your docs here.
  */
  public BalanceRobot( DriveSubsystem drive) {
    // addSequential(new DriveDistance(RobotPreferences.autoDriveDistance()));
    // addSequential(new DoDelay(RobotPreferences.autoDelay()));
    // addSequential(new ShooterUp());
    // addCommands(new DriveDistance(drive,12,.25));
    addCommands(new DriveToPitch(drive, .8, 10));
    addCommands(new WaitCommand(2));
    addCommands( new DriveToPitch(drive, .5, -5));
    addCommands(new WaitCommand(2));
    addCommands(new DriveToPitch(drive, .5, 0, true));
    
   
  }
}
  

// The code you typed in **RobotPreferences.java** should look like this

// '''java
// public static double autoDelay() {
//   return Preferences.getInstance().getDouble("autoDelay", 5.0);
// }

// public static double autoDriveDistance() {
//   return Preferences.getInstance().getDouble("autoDriveDistance", 12.0);
// }