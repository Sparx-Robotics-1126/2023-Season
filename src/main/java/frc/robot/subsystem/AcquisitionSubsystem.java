package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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


    // digital input limits
    private DigitalInput lowerLimitLeft;
    private DigitalInput lowerLimitRight;

    private DigitalInput upperLimitLeft;
    private DigitalInput upperLimitRight;

    // Pneumatic Compressors
    private Compressor pcmCompressor;

    private DoubleSolenoid DoublePCM;

    public AcquisitionSubsystem() {

        // _acquisitionsSensors = new DrivesSensors();
        xMotorLeft = new TalonSRX(AcquisitionConstants.ELEVATIONS_LEFT_MOTOR);
        TalonSRX xMotorRight = new TalonSRX(AcquisitionConstants.ELEVATIONS_RIGHT_MOTOR);

        yMotorLeft = new TalonSRX(AcquisitionConstants.EXTENDERS_LEFT_MOTOR);
        

        configureMotors(xMotorLeft, xMotorRight, yMotorLeft);

        // digital input limits
        lowerLimitLeft = new DigitalInput(0);
        lowerLimitRight = new DigitalInput(1);

        upperLimitLeft = new DigitalInput(2);
        upperLimitRight = new DigitalInput(3);

        // Compressors
        pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        DoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
        

    }
    public void elevate() {
        System.out.println("elevate");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PRESSURE", getPressure());
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
        config.peakOutputForward = AcquisitionConstants.MAX_MOTOR_POWER;
        config.peakOutputReverse = -AcquisitionConstants.MAX_MOTOR_POWER;

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

    public void xMoveTo(double meters) {
        xMotorLeft.set(ControlMode.Position, meters);
        xMotorRight.set(ControlMode.Position, meters);
    }

    public void yMoveTo(double meters) {
        yMotorLeft.set(ControlMode.Position, meters);
    }

    public void solonoidOff() {

        DoublePCM.set(Value.kOff);
    }

    public void solonoidForward() {

        DoublePCM.set(Value.kForward);
    }

    public void solonoidReverse() {

        DoublePCM.set(Value.kReverse);
    }

    /**
     * @return Average Y encoder position in meters.
     */
    public double getYPos() {
        return (yMotorLeft.getSelectedSensorPosition());
    }

    /**
     * @return Average X encoder position in meters.
     */
    public double getXPos() {
        return (xMotorLeft.getSelectedSensorPosition()
                + xMotorRight.getSelectedSensorPosition()) / 2;
    }

    public double getPressure() {

        return pcmCompressor.getPressure();
    }

    public void compressorDisable() {

         pcmCompressor.disable();
    }

    public void compressorEnable(){
        pcmCompressor.enableAnalog(AcquisitionConstants.MIN_PRESSURE, AcquisitionConstants.MAX_PRESSURE);
    }

    public void reset() {
        xMotorLeft.setSelectedSensorPosition(0);
        xMotorRight.setSelectedSensorPosition(0);
        yMotorLeft.setSelectedSensorPosition(0);
    }

    public void returnToHome() {
        // TODO: Set command to ReturnToHome
    }
}