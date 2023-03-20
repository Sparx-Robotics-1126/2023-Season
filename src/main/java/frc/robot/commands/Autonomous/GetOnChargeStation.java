package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystem.DriveSubsystem;

public class GetOnChargeStation  extends CommandBase {
    private final DriveSubsystem _drive;
    private final boolean _reversed = AutoConstants.BalanceAuto.REVERSED;
    private final double _finalAngle = AutoConstants.BalanceAuto.FINAL_ANGLE;

    public GetOnChargeStation(DriveSubsystem drive) {
        _drive =drive;
        addRequirements(drive);

    }

    @Override
    public void initialize() {
        double DRIVE_SPEED = AutoConstants.BalanceAuto.DRIVE_SPEED;
        double driveSpeed = (_reversed ? -1 : 1) * DRIVE_SPEED;

        _drive.arcadeDrive(driveSpeed, 0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        _drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return (_reversed ? -1 : 1) * _drive.getPitch() > _finalAngle;
    }
}
