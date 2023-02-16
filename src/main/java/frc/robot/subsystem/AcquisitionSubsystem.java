package frc.robot.subsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.drives.DrivesSensorInterface;
import frc.drives.DrivesSensors;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class AcquisitionSubsystem extends SubsystemBase {

    private DrivesSensors _acquisitionsSensors;

    // motors for elevations (X) and extenders (Y).
    TalonSRX xMotor;
    TalonSRX xMotorRight; // slave of xMotor
    TalonSRX yMotor;
    TalonSRX yMotorRight; // slave of yMotor

    public AcquisitionSubsystem() {

        xMotor = new TalonSRX(Constants.ELEVATIONS_LEFT_MOTOR);
        TalonSRX xMotorSlave = new TalonSRX(Constants.ELEVATIONS_RIGHT_MOTOR);

        xMotor.follow(xMotorSlave);
        configureMotor(xMotor, xMotorSlave);

        yMotor = new TalonSRX(Constants.EXTENDERS_LEFT_MOTOR);
        TalonSRX yMotorSlave = new TalonSRX(Constants.EXTENDERS_RIGHT_MOTOR);

        yMotor.follow(yMotorSlave);
        configureMotor(xMotor, yMotor);
    }

    public void elevate() {
        System.out.println("elevate");
    }

    public void periodic() {

    }

    private static void configureMotor(TalonSRX master, TalonSRX... slavesSrxs) {
        master.configAllSettings(new TalonSRXConfiguration());
        // speed command
        master.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        master.configPeakCurrentLimit(Constants.MAX_CURRENT);

        for (TalonSRX slave : slavesSrxs) {
            slave.configAllSettings(new TalonSRXConfiguration());
            // set speed command
            slave.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
            slave.configPeakCurrentLimit(Constants.MAX_CURRENT);

        }
    }

}