package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.Constants.AcquisitionConstants;

public class MoveTo extends CommandBase {
    private AcquisitionSubsystem acquisition;

    /**
     * In meters.
     */
    private double xPos;

    /**
     * In meters.
     */
    private double yPos;
    
    public MoveTo(AcquisitionSubsystem acquisition, double xPosition, double yPosition){
        if (xPosition < 0 || xPosition > AcquisitionConstants.X_MAX_METERS
            || yPosition < 0 || yPosition > AcquisitionConstants.Y_MAX_METERS)
            throw new IllegalArgumentException("Invalid positions.");

        this.acquisition = acquisition;
        xPos = xPosition;
        yPos = yPosition;
    }

    @Override
    public void initialize()
    {
        acquisition.xMoveTo(xPos);
        acquisition.yMoveTo(yPos);
    }

    @Override
    public boolean isFinished() {
        // TalonSRXs should be closed loop and stop themselves automatically, this is just to satisfy the command framework.
        return (Math.abs(xPos - acquisition.getXPos()) <= AcquisitionConstants.POSITION_EPSILON_METERS
            && Math.abs(yPos - acquisition.getYPos()) <= AcquisitionConstants.POSITION_EPSILON_METERS);
    }
}
