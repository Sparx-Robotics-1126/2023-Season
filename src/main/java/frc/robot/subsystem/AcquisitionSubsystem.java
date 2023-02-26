package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;
import frc.sensors.DrivesSensors;
import frc.robot.Constants.AcquisitionConstants;

public class AcquisitionSubsystem extends SubsystemBase {

    private DrivesSensors _acquisitionsSensors;

    // motors for elevations (X) and extenders (Y).
    private TalonSRX xMotorLeft;
    private TalonSRX xMotorRight; // slave of xMotor

    private TalonSRX yMotorLeft;
    private TalonSRX yMotorRight; // slave of yMotor

    //digital input limits
    private DigitalInput lowerLimitLeft;
    private DigitalInput lowerLimitRight;

    private DigitalInput upperLimitLeft;
    private DigitalInput upperLimitRight;

    public AcquisitionSubsystem() {

        xMotorLeft = new TalonSRX(AcquisitionConstants.ELEVATIONS_LEFT_MOTOR);
        TalonSRX xMotorRight = new TalonSRX(AcquisitionConstants.ELEVATIONS_RIGHT_MOTOR);

        xMotorLeft.follow(xMotorRight);
        configureMotor(xMotorLeft, xMotorRight);

        yMotorLeft = new TalonSRX(AcquisitionConstants.EXTENDERS_LEFT_MOTOR);
        TalonSRX yMotorLeft = new TalonSRX(AcquisitionConstants.EXTENDERS_RIGHT_MOTOR);

        yMotorLeft.follow(yMotorLeft);
        configureMotor(xMotorLeft, yMotorLeft);

        //digital input limits
        lowerLimitLeft = new DigitalInput(0);
        lowerLimitRight = new DigitalInput(1);

        upperLimitLeft = new DigitalInput(2);
        upperLimitRight = new DigitalInput(3);
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


    public void setXMotorLeft(double speed) {
        xMotorLeft.set(ControlMode.PercentOutput, speed);
    }

    public void setXMotorRight(double speed) {
        xMotorRight.set(ControlMode.PercentOutput, speed);
    }

    public void setYMotorLeft(double speed) {
        yMotorLeft.set(ControlMode.PercentOutput, speed);
    }

    public void setYMotorRight(double speed) {
        yMotorRight.set(ControlMode.PercentOutput, speed);
    }


    public boolean getLowerLimitLeft() {
        return lowerLimitLeft.get();
    }

    public boolean getLowerLimitRight() {
        return lowerLimitLeft.get();
    }

    public boolean getUpperLimitLeft() {
        return upperLimitLeft.get();
    }

    public boolean getUpperLimitRight() {
        return upperLimitRight.get();
    }


    public void returnToHome() {

    }


}