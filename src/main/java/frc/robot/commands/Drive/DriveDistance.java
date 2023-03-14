package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem m_drive;

  private double distance;
  private double m_speed;
  private boolean isReverse;
  private double startAngle;

  /**
   * Creates a new DriveDistance.
   *
   * @param distanceInMeters The number of meters the robot will drive
   * @param speed            The speed at which the robot will drive
   * @param drive            The drive subsystem on which this command will run
   */
  public DriveDistance(DriveSubsystem drive, double distanceInMeters, double speed) {
    this.distance = distanceInMeters;
  
    isReverse = false;
    m_drive = drive;
    if (distanceInMeters < 0){
      isReverse = true;
      m_speed = speed;
    }
    else {
      m_speed = -speed;
    }
 
    addRequirements(drive);

  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();
    startAngle = m_drive.getHeading();
  }

  @Override
  public void execute() {
    double turnValue = 0;
    double currentAngle = m_drive.getHeading();

    if (currentAngle > startAngle + 2) {

      turnValue = 0.3;

    } else if (currentAngle > startAngle + 0.5) {
      turnValue = 0.2;
    } else if (currentAngle < startAngle - 0.5) {
      turnValue = -0.2;
    } else if (currentAngle < startAngle - 2) {
      turnValue = -0.3;
    } else {
      turnValue = 0;
    }
    double moveValue = m_speed;

    m_drive.arcadeDrive(moveValue, -turnValue);

  }

  /**
   * @param interrupted
   */
  @Override
  public void end(boolean interrupted) {
    // m_drive.stop();
    m_drive.arcadeDrive(0, 0);
  }

  /**
   * @return boolean
   */
  @Override
  public boolean isFinished() {
    // var finished = false;

    var actualDistance = -m_drive.getAverageEncoderDistance();
  
    if (isReverse) {
      if (actualDistance < distance) {
        return true;
      }
    } else {
      if (actualDistance > distance) {
        return true;
      }

    }
    return false;

    // if (finished) {
    // System.out.println("Drive Distance " + distance + " finished");
    // }
    // return finished;
  }
}
