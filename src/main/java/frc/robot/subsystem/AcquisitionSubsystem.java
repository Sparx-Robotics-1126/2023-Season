package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import static frc.robot.Constants.AcquisitionConstants.*;

public class AcquisitionSubsystem extends SubsystemBase {

    // private DrivesSensors _acquisitionsSensors;

    // motors for elevations (X) and extenders (Y).
    private TalonSRX xMotor;

    private TalonSRX yMotorLeft;
    private TalonSRX yMotorRight;

    private Encoder lowerEncoderRight;
    private Encoder lowerEncoderLeft;

    private Encoder upperEncoder;

    // digital input limits
    private DigitalInput lowerLimitLeft;
    private DigitalInput lowerLimitRight;

    private DigitalInput upperLimit;

    // Pneumatic Compressors
    private Compressor pcmCompressor;

    private DoubleSolenoid DoublePCM;

    public AcquisitionSubsystem() {

        // _acquisitionsSensors = new DrivesSensors();
        xMotor = new TalonSRX(EXTENDERS_MOTOR);

        yMotorLeft = new TalonSRX(ELEVATIONS_LEFT_MOTOR);
        yMotorRight = new TalonSRX(ELEVATIONS_RIGHT_MOTOR);

        configureMotors(yMotorLeft, yMotorRight, xMotor);

        // Encoders
        lowerEncoderLeft = new Encoder(ELEVATIONS_LEFT_ENCODER_A, ELEVATIONS_LEFT_ENCODER_B);
        lowerEncoderRight = new Encoder(ELEVATIONS_RIGHT_ENCODER_A, ELEVATIONS_RIGHT_ENCODER_B);
        upperEncoder = new Encoder(EXTENDERS_ENCODER_A, EXTENDERS_ENCODER_B);

        configureEncoders(lowerEncoderLeft, lowerEncoderRight, upperEncoder);

        // digital input limits
        lowerLimitLeft = new DigitalInput(ELEVATIONS_LEFT_LIMIT);
        lowerLimitRight = new DigitalInput(ELEVATIONS_RIGHT_LIMIT);

        upperLimit = new DigitalInput(EXTENDERS_LIMIT);

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
        SmartDashboard.putNumber("ENCODER_POS", upperEncoder.getDistance());
    }

    private static void configureEncoders(Encoder... encoders) {
        for (Encoder encoder : encoders)
            encoder.setDistancePerPulse(PULSES_TO_METERS);
    }

    private static void configureMotors(TalonSRX... controllers) {
        TalonSRXConfiguration config = new TalonSRXConfiguration();

        config.voltageCompSaturation = Constants.NOMINAL_VOLTAGE;
        config.peakCurrentLimit = Constants.MAX_CURRENT;
        config.peakOutputForward = MAX_MOTOR_POWER;
        config.peakOutputReverse = -MAX_MOTOR_POWER;

        for (TalonSRX controller : controllers)
            controller.configAllSettings(config);
    }

    public void setYMotorLeft(double power) {
        yMotorLeft.set(ControlMode.PercentOutput, power);
    }

    public void setYMotorRight(double power) {
        yMotorRight.set(ControlMode.PercentOutput, power);
    }

    public void setXMotor(double power) {
        xMotor.set(ControlMode.PercentOutput, power);
    }

    public boolean getLowerLimitLeft() {
        return lowerLimitLeft.get();
    }

    public boolean getLowerLimitRight() {
        return lowerLimitRight.get();
    }

    public boolean getUpperLimit() {
        return upperLimit.get();
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
     * @return The Y encoder position in meters.
     */
    public double getYPos() {
        return upperEncoder.getDistance();
    }

    /**
     * @return Average X encoder position in meters.
     */
    public double getXPos() {
        return (lowerEncoderLeft.getDistance()
            + lowerEncoderRight.getDistance()) / 2;
    }

    public double getPressure() {

        return pcmCompressor.getPressure();
    }

    public void compressorDisable() {

         pcmCompressor.disable();
    }

    public void compressorEnable(){
        pcmCompressor.enableAnalog(MIN_PRESSURE, MAX_PRESSURE);
    }

    public void reset() {
        lowerEncoderLeft.reset();
        lowerEncoderRight.reset();
        upperEncoder.reset();
    }

    public void returnToHome() {
        // TODO: Set command to ReturnToHome
    }
}