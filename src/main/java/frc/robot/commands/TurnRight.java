package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnRight extends CommandBase{

   
    DriveSubsystem m_drive;
    double m_turnAngle;
double m_finalAngle;

    public TurnRight(DriveSubsystem drive, double turnAngle) {
        m_drive = drive;
        m_turnAngle = turnAngle;

        m_finalAngle = m_drive.getRotation + turnAngle;


        addRequirements(drive);
    }

    @Override
    public void execute() {
        distanceToStop = Math.abs(m_finalAngle - m_drive.getRotation);

		if (m_drive.getRotation >= m_finalAngle)
			return m_drive.tankDrive(0,0);
		else if (distanceToStop <= stopAngle)
		{
			speedToStop = distanceToStop / stopAngle;

			//Slow down the robot when we near the desired angle to avoid overshooting it.
			if (speedToStop < minSpeed)
				speedToStop = minSpeed;

			return m_drive.tankDrive(speedToStop,-speedToStop) ;
		}
			
		return new DrivesOutput(SPEED, -SPEED);
    }
}
