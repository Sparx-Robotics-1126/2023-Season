package frc.robot.subsystem;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import static frc.robot.Constants.AcquisitionConstants.*;

public class AcquisitionSubsystem extends ShuffleSubsystem {
    // Motors for extenders/reach (X) and elevations (Y)
    private TalonSRX xMotor;
    private CANSparkMax yMotor;

    // Motor encoders
    private Encoder xEncoder;
    private RelativeEncoder yEncoder;

    // Limit switches
    private DigitalInput xLimit;
    private DigitalInput yLimit;

    // Pneumatics
    @SuppressWarnings("unused")
    private Compressor compressor;
    private Solenoid grabberSolenoid;

    private PIDController xController;
    private PIDController yController;

    private double xPower;
    private double yPower;

    private double prevXPower;
    private double prevYPower;

    public AcquisitionSubsystem() {
        xMotor = new WPI_TalonSRX(X_MOTOR);
        yMotor = new CANSparkMax(Y_MOTOR, MotorType.kBrushless);

        TalonSRXConfiguration xConfig = new TalonSRXConfiguration();

        xConfig.voltageCompSaturation = Constants.NOMINAL_VOLTAGE;
        xConfig.peakCurrentLimit = Constants.MAX_CURRENT;
        xConfig.peakOutputForward = X_MAX_MOTOR_POWER;
        xConfig.peakOutputReverse = -X_MAX_MOTOR_POWER;
        
        xMotor.configAllSettings(xConfig);
        xMotor.setNeutralMode(NeutralMode.Coast);

        yMotor.restoreFactoryDefaults();
        yMotor.setIdleMode(IdleMode.kCoast);
        yMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        yMotor.setSmartCurrentLimit((int) Math.round(Constants.MAX_CURRENT * Y_MAX_MOTOR_POWER));

        // Encoders
        xEncoder = new Encoder(X_ENCODER_A, X_ENCODER_B);
        yEncoder = yMotor.getEncoder();

        xEncoder.setDistancePerPulse(X_METERS_PER_PULSE);
        yEncoder.setPositionConversionFactor(Y_METERS_PER_PULSE);

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

        xPower = 0;
        yPower = 0;

        prevXPower = 0;
        prevYPower = 0;
    }

    @Override
    public void periodic() {
        double xOut = X_FEEDFORWARD;
        double yOut = Y_FEEDFORWARD;

        boolean xUsePID = false;
        boolean yUsePID = false;

        if (xPower != 0) {
            // We have a manual power command. Do it.
            xOut += xPower;
            prevXPower = xPower;
        } else if (prevXPower != 0) {
            // We don't have a power command, but are just returning from one.
            // Set the X setpoint to the current position so we can resume PID
            // and hold our current position.
            xMoveTo(getXPos());
            prevXPower = 0;
            xUsePID = true;
        } else {
            // We don't have a power command and aren't returning from one.
            // Let PID take over.
            xUsePID = true;
        }

        // Do the same thing for y.
        if (yPower != 0) {
            yOut += yPower;
            prevYPower = yPower;
        } else if (prevYPower != 0) {
            yMoveTo(getYPos());
            prevYPower = 0;
            yUsePID = true;
        } else {
            yUsePID = true;
        }

        if (xUsePID)
            if (xController.getSetpoint() == 0)
                xOut -= RETURN_HOME_POWER;
            else 
                xOut += xController.calculate(getXPos());
        
        if (yUsePID)
            if (yController.getSetpoint() == 0)
                yOut -= RETURN_HOME_POWER;
            else
                yOut += yController.calculate(getYPos());

        if (xLimit.get() && xOut <= 0) {
            xEncoder.reset();
            xMotor.set(ControlMode.PercentOutput, -X_TENSION_POWER);
        } else if (getXPos() < X_MAX)
            xMotor.set(ControlMode.PercentOutput, xOut);
        else
            xMotor.set(ControlMode.PercentOutput, -RETURN_HOME_POWER);

        if (yLimit.get() && yOut <= 0) {
            yEncoder.setPosition(0);
            yMotor.set(0);
        } else if (getYPos() < Y_MAX)
            yMotor.set(yOut);
        else
            yMotor.set(-RETURN_HOME_POWER);
        
        SmartDashboard.putNumber("yOut", yOut);
        SmartDashboard.putNumber("xOut", xOut);

        SmartDashboard.putNumber("Y_POS", getYPos());
        SmartDashboard.putNumber("X_POS", getXPos());

        SmartDashboard.putBoolean("X_LIMIT", xLimit.get());
        SmartDashboard.putBoolean("Y_LIMIT", yLimit.get());

        SmartDashboard.putNumber("Y_SETPOINT", yController.getSetpoint());
        SmartDashboard.putNumber("X_SETPOINT", xController.getSetpoint());
    }

    public void grabberClose() {
        grabberSolenoid.set(false);
    }

    public void grabberOpen() {
        grabberSolenoid.set(true);
    }
    public boolean getGrabberState()
    {
        return grabberSolenoid.get();
    }
    /**
     * @return The average Y encoder position in meters.
     */
    public double getYPos() {
        return yEncoder.getPosition();
    }

    /**
     * @return The X encoder position in meters.
     */
    public double getXPos() {
        return xEncoder.getDistance();
    }

    public void setXPower(double power) {
        xPower = power;
    }

    public void setYPower(double power) {
        yPower = power;
    }

    public void xMoveTo(double pos) {
        if (pos >= 0 && pos <= X_MAX) {
            xController.reset();
            xController.setSetpoint(pos);
        }
    }

    public void yMoveTo(double pos) {
        if (pos >= 0 && pos <= Y_MAX) {
            yController.reset();
            yController.setSetpoint(pos);
        }
    }

    public boolean atSetpoint() {
        return xController.atSetpoint() && yController.atSetpoint();
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub

    }

    @Override
    public void displayShuffleboard() {

    }

    @Override
    public void tuningInit() {

        // xController = new PIDController(X_MOTOR_P, X_MOTOR_I, X_MOTOR_D);
        // yController = new PIDController(Y_MOTOR_P, Y_MOTOR_I, Y_MOTOR_D);

        SmartDashboard.putNumber("Extention kP", X_MOTOR_P);
        SmartDashboard.putNumber("Extention kI", X_MOTOR_I);
        SmartDashboard.putNumber("Extention kD", X_MOTOR_D);
        SmartDashboard.putNumber("Extention Max", X_MAX);
        SmartDashboard.putNumber("Extention Max Power", X_MAX_MOTOR_POWER);

        SmartDashboard.putNumber("Elevator kP", Y_MOTOR_P);
        SmartDashboard.putNumber("Elevator", Y_MOTOR_I);
        SmartDashboard.putNumber("Elevator", Y_MOTOR_D);
        SmartDashboard.putNumber("Elevator Max", Y_MAX);
        SmartDashboard.putNumber("Elevator Max Power", Y_MAX_MOTOR_POWER);
        SmartDashboard.putNumber("Elevator Feedforward", Y_FEEDFORWARD);
    }

    @Override
    public void tuningPeriodic() {
      

        X_MOTOR_P = SmartDashboard.getNumber("Extention kP", X_MOTOR_P);
        X_MOTOR_I =SmartDashboard.getNumber("Extention kI", X_MOTOR_I);
        X_MOTOR_D =SmartDashboard.getNumber("Extention kD", X_MOTOR_D);
        X_MAX = SmartDashboard.getNumber("Extention Max", X_MAX);
        X_MAX_MOTOR_POWER=SmartDashboard.getNumber("Extention Max Power", X_MAX_MOTOR_POWER);


        Y_MOTOR_P =SmartDashboard.getNumber("Elevator kP", Y_MOTOR_P);
        Y_MOTOR_I =SmartDashboard.getNumber("Elevator", Y_MOTOR_I);
        Y_MOTOR_D =SmartDashboard.getNumber("Elevator", Y_MOTOR_D);
        Y_MAX = SmartDashboard.getNumber("Elevator Max", Y_MAX);
        Y_MAX_MOTOR_POWER = SmartDashboard.getNumber("Elevator Max Power", Y_MAX_MOTOR_POWER);
        Y_FEEDFORWARD = SmartDashboard.getNumber("Elevator Feedforward", Y_FEEDFORWARD);
    }
}
