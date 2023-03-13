package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.commands.HoldPosition;

import frc.robot.Constants;
import static frc.robot.Constants.AcquisitionConstants.*;

public class AcquisitionSubsystem extends SubsystemBase 
{
    // Motors for elevations (X) and extenders (Y).
    private TalonSRX xMotor;

    private TalonSRX yMotorLeft;
    private TalonSRX yMotorRight;

    private Encoder yEncoderRight;
    private Encoder yEncoderLeft;

    private Encoder xEncoder;

    // Limit switches
    private DigitalInput yLimitLeft;
    private DigitalInput yLimitRight;

    private DigitalInput xLimit;

    // Pneumatics
    private Compressor compressor;

    private Solenoid grabberSolenoid;

    public AcquisitionSubsystem() 
    {
        xMotor = new TalonSRX(X_MOTOR);

        yMotorLeft = new TalonSRX(Y_LEFT_MOTOR);
        yMotorRight = new TalonSRX(Y_RIGHT_MOTOR);

        yMotorRight.setInverted(true);

        configureMotors(yMotorLeft, yMotorRight, xMotor);

        // Encoders
        yEncoderLeft = new Encoder(Y_LEFT_ENCODER_A, Y_LEFT_ENCODER_B);
        yEncoderRight = new Encoder(Y_RIGHT_ENCODER_A, Y_RIGHT_ENCODER_B);
        xEncoder = new Encoder(X_ENCODER_A, X_ENCODER_B);

        configureEncoders(yEncoderLeft, yEncoderRight, xEncoder);

        // Limit switches
        yLimitLeft = new DigitalInput(Y_LEFT_LIMIT);
        yLimitRight = new DigitalInput(Y_RIGHT_LIMIT);

        xLimit = new DigitalInput(X_LIMIT);

        // Pneumatics
        compressor = new Compressor(COMPRESSOR, PneumaticsModuleType.CTREPCM);
        grabberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);

        setDefaultCommand(new HoldPosition(this));
    }

    private static void configureEncoders(Encoder... encoders) 
    {
        for (Encoder encoder : encoders)
            encoder.setDistancePerPulse(PULSES_TO_METERS);
    }

    private static void configureMotors(TalonSRX... controllers) 
    {
        TalonSRXConfiguration config = new TalonSRXConfiguration();

        config.voltageCompSaturation = Constants.NOMINAL_VOLTAGE;
        config.peakCurrentLimit = Constants.MAX_CURRENT;
        config.peakOutputForward = MAX_MOTOR_POWER;
        config.peakOutputReverse = -MAX_MOTOR_POWER;

        for (TalonSRX controller : controllers)
            controller.configAllSettings(config);
    }

    public void setYMotorLeft(double power) 
    {
        yMotorLeft.set(ControlMode.PercentOutput, power);
    }

    public void setYMotorRight(double power) 
    {
        yMotorRight.set(ControlMode.PercentOutput, power);
    }

    public void setXMotor(double power) 
    {
        xMotor.set(ControlMode.PercentOutput, power);
    }

    public boolean getYLimitLeft() 
    {
        return yLimitLeft.get();
    }

    public boolean getYLimitRight() 
    {
        return yLimitRight.get();
    }

    public boolean getXLimit() 
    {
        return xLimit.get();
    }

    public void grabberClose() 
    {
        grabberSolenoid.set(false);
    }

    public void grabberOpen() 
    {
        grabberSolenoid.set(true);
    }

    /**
     * @return The Y encoder position in meters.
     */
    public double getYPos() 
    {
        return (yEncoderLeft.getDistance()
            + yEncoderRight.getDistance()) / 2;
    }

    /**
     * @return Average X encoder position in meters.
     */
    public double getXPos() 
    {
        return xEncoder.getDistance();
    }

    public double getPressure() 
    {
        return compressor.getPressure();
    }

    public void compressorDisable() 
    {
        compressor.disable();
    }

    public void reset() 
    {
        yEncoderLeft.reset();
        yEncoderRight.reset();
        xEncoder.reset();
    }
}
