package frc.robot.commands;

// import frc.robot.Constants.DriveConstants;
import frc.robot.subsystem.DriveSubsystem;
// import frc.robot.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToPitch extends CommandBase {
    private final DriveSubsystem m_drive;
    private double _pitch;
    private double _startAngle;
    private double _speed;
    private boolean isReverse;

    public DriveToPitch(DriveSubsystem driveSubsystem, double speed, double pitch) {
        m_drive = driveSubsystem;
        _pitch = pitch;
         _speed = speed;
        isReverse = false;

        if (pitch < 0) {
            isReverse = false;
           
        } else {
            isReverse = true;
           
        }

        if (speed <0){
            _speed = +speed;
        }
        else {
            _speed = -speed;
        }

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
        _startAngle = m_drive.getHeading();
    }

    @Override
    public void execute() {
        double turnValue = 0;
        double currentAngle = m_drive.getHeading();
        if (currentAngle > _startAngle + 2) {
            turnValue = 0.3;

        } else if (currentAngle > _startAngle + 0.5) {
            turnValue = 0.2;
        } else if (currentAngle < _startAngle - 0.5) {
            turnValue = -0.2;
        } else if (currentAngle < _startAngle - 2) {
            turnValue = -0.3;
        } else {
            turnValue = 0;
        }
        double moveValue = _speed;

        m_drive.arcadeDrive(moveValue, turnValue);

        // m_drive.tankDrive(DriveConstants.kAutoDriveForwardSpeed,
        // DriveConstants.kAutoDriveForwardSpeed);
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
        // System.out.println("DriveForwardCmd finished!");
        var currentPitch = m_drive.getPitch();
        if (isReverse) {
            if (currentPitch > _pitch) {
                System.out.println(">Drive to Pitch " + _pitch + " finished! "+ currentPitch  );
                return true;
            } else {
                return false;
            }
        } else {
            if (currentPitch < _pitch) {
                System.out.println("<Drive to Pitch " + _pitch + " finished!"+ currentPitch);
                return true;
            } else {
                return false;
            }
        }

    }
}
