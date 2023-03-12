package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystem.DriveSubsystem;

public class TurnToRelativeAngle extends PIDCommand {
    private final DriveSubsystem drive;
  
   
    private final PIDController controller;
    /**
     * Turns to robot to the specified angle.
     *
     * @param targetAngleDegrees The angle to turn to
     * @param drive              The drive subsystem to use
     */
    public TurnToRelativeAngle(double targetRelativeAngleDegrees, DriveSubsystem drive) {
      super(
          new PIDController(DriveConstants.kRelTurnP, DriveConstants.kRelTurnI, DriveConstants.kRelTurnD),
          // Close loop on heading
          drive::getRotation,
          // Set reference to target
          targetRelativeAngleDegrees,
          // Pipe output to turn robot
        output -> {
                drive.arcadeDrive(0, output);
                SmartDashboard.putNumber("OUTPUT", output);
        },
          // Require the drive
          drive);
      this.drive = drive;
      this.controller = getController();
      // Set the controller to be continuous (because it is an angle controller)
      controller.enableContinuousInput(-180, 180);
      // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
      // setpoint before it is considered as having reached the reference
      controller.setTolerance(DriveConstants.kRelTurnToleranceDeg, DriveConstants.kRelTurnRateToleranceDegPerS);
    }
  
    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return controller.atSetpoint();
    }
  
  //   public double getPositionError() {
  //     return controller.getPositionError();
  //   }
  
  //   public double getVelocityError() {
  //     return controller.getVelocityError();
  //   }
   }