package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystem.DriveSubsystem;

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        // Close loop on heading
        () -> drive.getRotation(),
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output ->  {
          if (output > 0){
            drive.arcadeDrive(0, output +.035 );
          } else if (output <0){
            drive.arcadeDrive(0, output-.035 );
          } else{
            drive.arcadeDrive(0, output );
          }
          //drive.arcadeDrive(0, output );
          SmartDashboard.putNumber("OUTPUT", output);
  },
        // Require the drive
        drive);

        // drive.resetEncoders();
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg);
  }

  
  /** 
   * @return boolean
   */
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}