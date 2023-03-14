package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystem.DriveSubsystem;

public class TurnPID  extends PIDCommand {
    public TurnPID(PIDController pid, double targetAngleDegrees, DriveSubsystem drive) {
        super(pid,
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
        drive
        
        );
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
