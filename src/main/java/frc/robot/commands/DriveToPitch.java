package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystem.DriveSubsystem;
// import frc.robot.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToPitch extends CommandBase {
    private final DriveSubsystem _driveSubsystem;
    private final double _pitch;

    public DriveToPitch(DriveSubsystem driveSubsystem, double pitch) {
        _driveSubsystem = driveSubsystem;
        _pitch = pitch;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // System.out.println("DriveForwardCmd started!");
    }

    @Override
    public void execute() {
       // System.out.println("DriveForwardCmd executing!");

       if (_driveSubsystem.getPitch() > _pitch){
        _driveSubsystem.tankDrive(0, 0);
        end(true);
       }

       _driveSubsystem.tankDrive(DriveConstants.kAutoDriveForwardSpeed, DriveConstants.kAutoDriveForwardSpeed);
    }

    // @Override
    // public void end(boolean interrupted) {
    //    // System.out.println("DriveForwardCmd ended!");
    //    _driveSubsystem.setMotors(0, 0);
    //     System.out.println("DriveForwardCmd ended!");
    // }

    // @Override
    // public boolean isFinished() {
    //   //  System.out.println("DriveForwardCmd finished!");
    //     if (_driveSubsystem.getPitch() > _pitch)
    //         return true;
    //     else
    //         return false;
    // }
}
