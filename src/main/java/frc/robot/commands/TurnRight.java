package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveSubsystem;

public class TurnRight extends CommandBase{
   
	
	private double minSpeed = 0.3;
	private double finalAngle = 0;
	private double stopAngle = 90;
	private double speedToStop = 0;
	private double distanceToStop = 0;
   
    DriveSubsystem m_drive;
    double m_turnAngle;
double m_finalAngle;

    /**
     * @param drive
     * @param turnAngle
     */
    public TurnRight(DriveSubsystem drive, double turnAngle) {
        m_drive = drive;
        m_turnAngle = turnAngle;
       

        m_finalAngle = m_drive.getRotation() + turnAngle;
        minSpeed = 0.3;
        finalAngle = 0;
        stopAngle = 90;
         speedToStop = 0;
       distanceToStop = 0;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        distanceToStop = Math.abs(m_finalAngle - m_drive.getRotation());

		if (m_drive.getRotation() >= m_finalAngle)
			 m_drive.tankDrive(0,0);
		else if (distanceToStop <= stopAngle)
		{
			speedToStop = distanceToStop / stopAngle;

			//Slow down the robot when we near the desired angle to avoid overshooting it.
			if (speedToStop < minSpeed)
				speedToStop = minSpeed;

			 m_drive.tankDrive(speedToStop,-speedToStop) ;
		}
			
        m_drive.tankDrive(.3, -.3);
    }
}
