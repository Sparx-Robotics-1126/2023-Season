package frc.robot.commands.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystem.DriveSubsystem;

    public class BalanceOnChargeStation extends CommandBase {
        private final DriveSubsystem _drive;
        private final double POSITION_TOLERANCE = AutoConstants.BalanceAuto.POSITION_TOLERANCE;
        private final double VELOCITY_TOLERANCE = AutoConstants.BalanceAuto.VELOCITY_TOLERANCE;
        private final boolean IS_REVERSED = AutoConstants.BalanceAuto.REVERSED;
        private final double MAX_SPEED = AutoConstants.BalanceAuto.MAX_SPEED;
      
      
        private final PIDController pidController = new PIDController(
            AutoConstants.BalanceAuto.KP,
            AutoConstants.BalanceAuto.KI,
            AutoConstants.BalanceAuto.KD
        );
      
        public BalanceOnChargeStation(DriveSubsystem drivetrain) {
            _drive = drivetrain;
          addRequirements(drivetrain);
          pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
          pidController.setSetpoint(0);
        }
      
        @Override
        public void initialize() {
          pidController.reset();
        }
      
        @Override
        public void execute() {
          double pitch = (IS_REVERSED ? -1 : 1) * _drive.getPitch();
          double pidResult = pidController.calculate(pitch);
          double speed = MathUtil.clamp(pidResult, -MAX_SPEED, MAX_SPEED);
          _drive.arcadeDrive(speed, 0);
        }
      
        @Override
        public void end(boolean interrupted) {
            _drive.arcadeDrive(0, 0);
        }
      
        @Override
        public boolean isFinished() {
          return pidController.atSetpoint();
        }
    }
