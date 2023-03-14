package frc.robot.commands.Acquisition;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystem.AcquisitionSubsystem;
import static frc.robot.Constants.AcquisitionConstants.*;

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
    
    public MoveTo(AcquisitionSubsystem acquisition, double xPosition, double yPosition) {
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
        return acquisition.atSetpoint();
    }
}
