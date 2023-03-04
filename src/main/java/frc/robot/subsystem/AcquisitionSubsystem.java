package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// import frc.robot.sensors.DrivesSensors;
import frc.robot.Constants;
import frc.robot.Constants.AcquisitionConstants;

public class AcquisitionSubsystem extends SubsystemBase {

    // private DrivesSensors _acquisitionsSensors;

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
        
        // _acquisitionsSensors = new DrivesSensors();
        xMotorLeft = new TalonSRX(AcquisitionConstants.ELEVATIONS_LEFT_MOTOR);
        TalonSRX xMotorRight = new TalonSRX(AcquisitionConstants.ELEVATIONS_RIGHT_MOTOR);

        yMotorLeft = new TalonSRX(AcquisitionConstants.EXTENDERS_LEFT_MOTOR);
        TalonSRX yMotorRight = new TalonSRX(AcquisitionConstants.EXTENDERS_RIGHT_MOTOR);

        configureMotors(xMotorLeft, xMotorRight, yMotorLeft, yMotorRight);

        //digital input limits
        lowerLimitLeft = new DigitalInput(0);
        lowerLimitRight = new DigitalInput(1);

        upperLimitLeft = new DigitalInput(2);
        upperLimitRight = new DigitalInput(3);
    }

    public void elevate() {
        System.out.println("elevate");
    }
    
    @Override
    public void periodic() {

    }

    private static void configureMotors(TalonSRX... controllers) {
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        TalonSRXPIDSetConfiguration pidConfig = new TalonSRXPIDSetConfiguration();

        pidConfig.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        // This will automatically convert the encoder's raw units to meters.
        pidConfig.selectedFeedbackCoefficient = AcquisitionConstants.TICKS_TO_METERS;
        config.primaryPID = pidConfig;
        config.voltageCompSaturation = Constants.NOMINAL_VOLTAGE;
        config.peakCurrentLimit = Constants.MAX_CURRENT;

        for (TalonSRX controller : controllers)
            controller.configAllSettings(config);
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
        return lowerLimitRight.get();
    }

    public boolean getUpperLimitLeft() {
        return upperLimitLeft.get();
    }

    public boolean getUpperLimitRight() {
        return upperLimitRight.get();
    }

    public void xMoveTo(double meters)
    {
        xMotorLeft.set(ControlMode.Position, meters);
        xMotorRight.set(ControlMode.Position, meters);
    }

    public void yMoveTo(double meters)
    {
        yMotorLeft.set(ControlMode.Position, meters);
        yMotorRight.set(ControlMode.Position, meters);
    }

    /**
     * @return Average Y encoder position in meters.
     */
    public double getYPos()
    {
        return (yMotorLeft.getSelectedSensorPosition() 
            + yMotorRight.getSelectedSensorPosition()) / 2;
    }

    /**
     * @return Average X encoder position in meters.
     */
    public double getXPos()
    {
        return (xMotorLeft.getSelectedSensorPosition() 
            + xMotorRight.getSelectedSensorPosition()) / 2;
    }

    public void reset()
    {
        xMotorLeft.setSelectedSensorPosition(0);
        xMotorRight.setSelectedSensorPosition(0);
        yMotorLeft.setSelectedSensorPosition(0);
        yMotorRight.setSelectedSensorPosition(0);
    }

    public void returnToHome() {
        // TODO: Set command to ReturnToHome
    }
}