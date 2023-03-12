package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystem.AcquisitionSubsystem;
import static frc.robot.Constants.AcquisitionConstants.*;

public class HoldPosition extends CommandBase
{
    private AcquisitionSubsystem acquisition;
    private PIDController xController;
    private PIDController yController;
    
    public HoldPosition(AcquisitionSubsystem acquisition)
    {
        this.acquisition = acquisition;

        xController = new PIDController(MOTOR_P, MOTOR_I, MOTOR_D);
        yController = new PIDController(MOTOR_P, MOTOR_I, MOTOR_D);
    }

    @Override
    public void initialize()
    {
        xController.reset();
        yController.reset();
        
        xController.setSetpoint(acquisition.getXPos());
        yController.setSetpoint(acquisition.getYPos());
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
}
