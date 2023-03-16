package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import frc.robot.Constants;

import static frc.robot.Constants.AcquisitionConstants.*;

public class AcquisitionSubsystem extends SubsystemBase {
    // Motors for extenders/reach (X) and elevations (Y)
    private TalonSRX xMotor;
    private TalonSRX yMotor;

    // Motor encoders
    private Encoder xEncoder;
    private Encoder yEncoderLeft;
    private Encoder yEncoderRight;

    // Limit switches
    private DigitalInput xLimit;
    private DigitalInput yLimit;

    static {
        SmartDashboard.putNumber("Y_POWER", 0.1);
        SmartDashboard.putBoolean("ENABLE", false);
    }


    // Pneumatics
    @SuppressWarnings("unused")
    private Compressor compressor;
    private Solenoid grabberSolenoid;

    private PIDController xController;
    private PIDController yController;

    public AcquisitionSubsystem() {
        xMotor = new WPI_TalonSRX(X_MOTOR);
        yMotor = new WPI_TalonSRX(Y_LEFT_MOTOR);
        TalonSRX yMotorSlave = new WPI_TalonSRX(Y_RIGHT_MOTOR);

        yMotorSlave.setInverted(InvertType.OpposeMaster);
        yMotorSlave.follow(yMotor);

        configureMotors(yMotorSlave, yMotor, xMotor);

       // Encoders
        xEncoder = new Encoder(X_ENCODER_A, X_ENCODER_B);
        yEncoderLeft = new Encoder(Y_LEFT_ENCODER_A, Y_LEFT_ENCODER_B);
        yEncoderRight = new Encoder(Y_RIGHT_ENCODER_A, Y_RIGHT_ENCODER_B);

        yEncoderRight.setReverseDirection(true);

        xEncoder.setDistancePerPulse(X_PULSES_TO_METERS);
        yEncoderLeft.setDistancePerPulse(Y_PULSES_TO_METERS);
        yEncoderRight.setDistancePerPulse(Y_PULSES_TO_METERS);
        
        // Limit switches
        xLimit = new DigitalInput(X_LIMIT);
        yLimit = new DigitalInput(Y_LIMIT);

        // Pneumatics
        compressor = new Compressor(COMPRESSOR, PneumaticsModuleType.CTREPCM);
        grabberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);

        xController = new PIDController(X_MOTOR_P, X_MOTOR_I, X_MOTOR_D);
        yController = new PIDController(Y_MOTOR_P, Y_MOTOR_I, Y_MOTOR_D);

        xController.setTolerance(POSITION_EPSILON_METERS);
        yController.setTolerance(POSITION_EPSILON_METERS);

        xMoveTo(0);
        yMoveTo(0);
    }

    @Override
    public void periodic() {
        if (SmartDashboard.getBoolean("ENABLE", false))
            yMotor.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Y_POWER", 0.1));
        else
            yMotor.set(ControlMode.PercentOutput, 0.1);

        /*if (xLimit.get() && xController.getSetpoint() == 0) {
            xEncoder.reset();
            xMotor.set(ControlMode.PercentOutput, 0);
        } else {
            if (xController.getSetpoint() == 0)
                xMotor.set(ControlMode.PercentOutput, -RETURN_HOME_POWER);
            else {
                double xOut = xController.calculate(getXPos());
                xMotor.set(ControlMode.PercentOutput, xOut);
            }
        }
        
        if (yLimit.get() && yController.getSetpoint() == 0) {
            yEncoderLeft.reset();
            yEncoderRight.reset();
            yMotor.set(ControlMode.PercentOutput, 0);
        } else {
            if (yController.getSetpoint() == 0)
                yMotor.set(ControlMode.PercentOutput, -RETURN_HOME_POWER + Y_FEEDFORWARD);
            else {
                double yOut = yController.calculate(getYPos()) + Y_FEEDFORWARD;
                yMotor.set(ControlMode.PercentOutput, yOut);
            }
        }*/

        SmartDashboard.putNumber("Y_POS", getYPos());
        SmartDashboard.putNumber("X_POS", getXPos());
    }

    private static void configureMotors(TalonSRX... controllers) {
        TalonSRXConfiguration config = new TalonSRXConfiguration();

        config.voltageCompSaturation = Constants.NOMINAL_VOLTAGE;
        config.peakCurrentLimit = Constants.MAX_CURRENT;
        config.peakOutputForward = MAX_MOTOR_POWER;
        config.peakOutputReverse = -MAX_MOTOR_POWER;

        for (TalonSRX controller : controllers)
        {
            controller.configAllSettings(config);
            controller.setNeutralMode(NeutralMode.Coast);
        }
    }

    public void grabberClose() {
        grabberSolenoid.set(false);
    }

    public void grabberOpen() {
        grabberSolenoid.set(true);
    }

    /**
     * @return The average Y encoder position in meters.
     */
    public double getYPos() {
        return (yEncoderLeft.getDistance()
                + yEncoderRight.getDistance()) / 2;
    }

    /**
     * @return The X encoder position in meters.
     */
    public double getXPos() {
        return xEncoder.getDistance();
    }

    public void xMoveTo(double pos) {
        if (pos < 0 || pos > X_MAX)
            throw new IllegalArgumentException("Invalid X position.");
        
        xController.reset();
        xController.setSetpoint(pos);
    }

    public void yMoveTo(double pos) {
        if (pos < 0 || pos > Y_MAX)
            throw new IllegalArgumentException("Invalid Y position.");
        
        yController.reset();
        yController.setSetpoint(pos);
    }

    public boolean atSetpoint() {
        return xController.atSetpoint() && yController.atSetpoint();
    }
}
