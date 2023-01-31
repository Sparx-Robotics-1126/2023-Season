// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.drives.DrivesSensors;
// import frc.robot.subsystem.DriveSubsystem;

// public class ForwardDistance extends CommandBase {
//     private final DrivesSensors m_drive;
//     private final double m_distance;
//     private final double m_speed;
//     private static final double DISTANCE_kP = 0.03;
//     private static final double DISTANCE_DEADBAND = 1; //2 inches.
//     /**
//      * Creates a new DriveDistance.
//      *
//      * @param inches The number of inches the robot will drive
//      * @param speed The speed at which the robot will drive
//      * @param drive The drive subsystem on which this command will run
//      */
//     public ForwardDistance(double inches, double speed, DrivesSensors driveSensors) {
      
//       //super(driveSensors);

      
      
//       m_distance = driveSensors.getAverageEncoderDistance() + inches;
//       m_speed = speed;
//       m_drive = driveSensors;
//       // addRequirements(m_drive);
//     }
  
//     @Override
//     public void initialize() {
//       // m_drive.resetEncoders();
//       // m_drive.tankDrive(m_speed, m_speed);
//     }
  
//     @Override
//     public void execute() {
//      //m_drive.tankDrive(m_speed,m_speed );

//      double distanceError = m_distance - m_drive.getAverageEncoderDistance();
// 		//double angleError = TARGET_ANGLE - getSensors().getGyroAngle();

// 		double leftSpeed, rightSpeed;
// 		leftSpeed = rightSpeed = distanceError * DISTANCE_kP * m_speed;

// 		if (leftSpeed > 1)
// 		{
// 			leftSpeed = 1;
// 			rightSpeed = 1;
// 		}

// 		/*double gyroOffset = angleError * GYRO_kP;

// 		if (gyroOffset < 0) //Too far left.
// 			leftSpeed -= gyroOffset;
// 		else
// 			rightSpeed += gyroOffset;*/

// 		if (m_drive.getAverageEncoderDistance() > m_distance - DISTANCE_DEADBAND)
// 			return m_drive.tankDrive(m_speed,m_speed );

// 		return new DrivesOutput(leftSpeed, rightSpeed);
//     }
  
//     @Override
//     public void end(boolean interrupted) {
//       m_drive.tankDrive(0, 0);
//     }
  
//     @Override
//     public boolean isFinished() {
//       return Math.abs(m_drive.getAverageEncoderDistance()) >= m_distance;
//     }
// }
