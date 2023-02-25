package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystem.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToPitch extends CommandBase {
    private final DriveSubsystem m_drive;
    private double m_toPitch;
    private double _startAngle;
    private double m_speed;
    private boolean m_driveBackwards;
    private boolean m_reverse = false;
    private Timer m_timer;

    public DriveToPitch(DriveSubsystem driveSubsystem, double speed, double pitch, boolean driveBackwards,
            boolean reverse) {
        m_drive = driveSubsystem;
        m_toPitch = pitch;
        m_speed = -speed;
        m_driveBackwards = driveBackwards;
        m_reverse = reverse;

        if (m_driveBackwards) {
            m_speed *= -1;
        }
        m_timer = new Timer();

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
        m_timer = new Timer();
        m_timer.start();
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
        double moveValue = m_speed;

        m_drive.arcadeDrive(moveValue, -turnValue);

    }

    /**
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
        m_drive.applyBrakes();
        ;
    }

    /**
     * @return boolean
     */
    @Override
    public boolean isFinished() {

        if (m_timer.hasElapsed(AutoConstants.kPitchTimeoutSeconds)) {
            return true;
        }

        var currentPitch = m_drive.getPitch();
        if (!m_reverse) {
            if (m_toPitch == 0 && currentPitch != 0) {
                return false;
            }
            if (m_toPitch > 0) {
                SmartDashboard.putString("BALANCE_STAGE", "Postive");
                if (currentPitch > m_toPitch) {
                    SmartDashboard.putString("BALANCE_STAGE", "Pitch Found");
                    return true;
                }
            }

            if (m_toPitch < 0 && currentPitch > 0) {
                SmartDashboard.putString("BALANCE_STAGE", "Negative");
                return false;
            }

            if (currentPitch > m_toPitch) {
                SmartDashboard.putString("BALANCE_STAGE", "Pitch Found");
                return true;
            }

        }

        if (m_reverse) {
            if (m_toPitch == 0 && currentPitch != 0) {
                return false;
            }
            if (m_toPitch < 0) {
                SmartDashboard.putString("BALANCE_STAGE", "Negative");
                if (currentPitch < m_toPitch) {
                    SmartDashboard.putString("BALANCE_STAGE", "Pitch Found");
                    return true;
                }
            }

            if (m_toPitch > 0 && currentPitch < 0) {
                SmartDashboard.putString("BALANCE_STAGE", "Postive");
                return false;
            }

            if (currentPitch < m_toPitch) {
                SmartDashboard.putString("BALANCE_STAGE", "Pitch Found");
                return true;
            }

        }

        if (m_toPitch == 0 && currentPitch == 0) {
            SmartDashboard.putString("BALANCE_STAGE", "Equal");
            return true;
        }

        return false;

    }

}
