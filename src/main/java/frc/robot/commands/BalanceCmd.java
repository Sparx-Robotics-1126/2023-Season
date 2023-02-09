package frc.robot.commands;

// import java.util.function.DoubleConsumer;
// import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
// import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.DriveSubsystem;

public class BalanceCmd extends PIDCommand {

    public BalanceCmd(DriveSubsystem driveSubsystem) {
        super(
                new PIDController(0, 0, 0),
                driveSubsystem::getPitch,
                0,
                output-> driveSubsystem.tankDrive(output, output),
                driveSubsystem);

                getController().enableContinuousInput(-0.5, 0.5);
                getController().setTolerance(-1,1);
        
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
    
}
