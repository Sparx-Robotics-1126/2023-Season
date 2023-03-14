package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveSubsystem;

public class TurnRight extends CommandBase {

    private double minSpeed = 0.3;
    // private double finalAngle = 0;
    // private double stopAngle = 90;
    private double speedToStop = 0;
    private double distanceToStop = 0;
    // private double startAngle = 0;
    private double speed = .6;

    DriveSubsystem m_drive;
    double m_turnAngle;
    double m_finalAngle;

    /**
     * @param drive
     * @param turnAngle
     */
    public TurnRight(double turnAngle, DriveSubsystem drive) {
        m_drive = drive;
        m_turnAngle = turnAngle;

        // startAngle = m_drive.getHeading();
        m_finalAngle = m_drive.getHeading() + turnAngle;
        SmartDashboard.putNumber("FINAL_ANGLE", m_finalAngle);
        minSpeed = 0.35;
        // finalAngle = 0;
        // stopAngle = 90;
        speedToStop = 0;
        distanceToStop = 0;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_finalAngle = m_drive.getHeading() + m_turnAngle;
        SmartDashboard.putNumber("FINAL_ANGLE", m_finalAngle);
    }

    @Override
    public void execute() {

        m_drive.tankDrive(-speed, speed);
    }

    @Override
    public boolean isFinished() {
        distanceToStop = Math.abs(m_finalAngle - m_drive.getHeading());
        SmartDashboard.putNumber("DISTANCE_TO_STOP", distanceToStop);

        if (m_drive.getHeading() >= m_finalAngle) {
            return true;
        } else if (distanceToStop <= m_finalAngle) {
            speedToStop = distanceToStop / m_finalAngle;

            // Slow down the robot when we near the desired angle to avoid overshooting it.
            if (speedToStop < minSpeed)
                speedToStop = minSpeed;

            m_drive.tankDrive(-speedToStop, speedToStop);
            return false;
        }

        m_drive.tankDrive(-speed, speed);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.tankDrive(0, 0);
    }

}
