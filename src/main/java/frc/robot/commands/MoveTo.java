package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystem.AcquisitionSubsystem;
import static frc.robot.Constants.AcquisitionConstants.*;

public class MoveTo extends CommandBase
{
    private AcquisitionSubsystem acquisition;
    private PIDController xController;
    private PIDController yController;

    /**
     * In meters.
     */
    private double xPos;

    /**
     * In meters.
     */
    private double yPos;
    
    public MoveTo(AcquisitionSubsystem acquisition, double xPosition, double yPosition)
    {
        if (xPosition < 0 || xPosition > X_MAX_METERS
            || yPosition < 0 || yPosition > Y_MAX_METERS)
            throw new IllegalArgumentException("Invalid positions.");

        this.acquisition = acquisition;
        xPos = xPosition;
        yPos = yPosition;

        xController = new PIDController(MOTOR_P, MOTOR_I, MOTOR_D);
        yController = new PIDController(MOTOR_P, MOTOR_I, MOTOR_D);

        xController.setTolerance(POSITION_EPSILON_METERS);
        yController.setTolerance(POSITION_EPSILON_METERS);
    }

    @Override
    public void initialize()
    {
        xController.reset();
        yController.reset();
        
        xController.setSetpoint(xPos);
        yController.setSetpoint(yPos);
    }

    @Override
    public void execute()
    {
        double xOut = xController.calculate(acquisition.getXPos());
        double yOut = yController.calculate(acquisition.getYPos()) + Y_FEEDFORWARD;

        acquisition.setXMotor(xOut);
        acquisition.setYMotorLeft(yOut);
        acquisition.setYMotorRight(yOut);
    }

    @Override
    public boolean isFinished() {
        return yController.atSetpoint() && xController.atSetpoint();
    }
}
