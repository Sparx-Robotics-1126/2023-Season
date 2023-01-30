package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.DriveSubsystem;

public class Autonomous extends SequentialCommandGroup {
  /**
  * Add your docs here.
  */
  public Autonomous( DriveSubsystem drive) {
    // addSequential(new DriveDistance(RobotPreferences.autoDriveDistance()));
    // addSequential(new DoDelay(RobotPreferences.autoDelay()));
    // addSequential(new ShooterUp());
    addCommands(new DriveDistance(12,.25,drive));
   
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