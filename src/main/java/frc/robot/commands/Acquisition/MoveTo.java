package frc.robot.commands.Acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystem.AcquisitionSubsystem;

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
    
    public MoveTo(double xPosition, double yPosition, AcquisitionSubsystem acquisition) {
        addRequirements(acquisition);

        this.acquisition = acquisition;
        xPos = xPosition;
        yPos = yPosition;
    }

    @Override
    public void initialize() {
        acquisition.xMoveTo(xPos);
        acquisition.yMoveTo(yPos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
