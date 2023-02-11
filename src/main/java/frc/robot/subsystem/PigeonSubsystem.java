package frc.robot.subsystem;
// package frc.subsystem;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2_Faults;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

import frc.robot.Constants;
// import com.ctre.phoenixpro.hardware.Pigeon2;

import com.ctre.phoenix.sensors.Pigeon2Configuration;

public class PigeonSubsystem extends SubsystemBase {
    private final WPI_Pigeon2 _pigeon;

    // private Pigeon2 _test;
    private static Pigeon2_Faults _pigeonFaults = new Pigeon2_Faults();
    // private BasePigeon m_basePigeon;

    public PigeonSubsystem() {
        _pigeon = new WPI_Pigeon2(Constants.Pigeon2ID);
        _pigeonFaults = new Pigeon2_Faults();
        // _test = new Pigeon2(4);
        initPigeon();
    }

    private void initPigeon() {
        var toApply = new Pigeon2Configuration();
        _pigeon.configAllSettings(toApply);

        // used by pro still thinking about it
        // _pigeon2.getConfigurator().apply(toApply);

        /* Speed up signals to an appropriate rate */
        // _pigeon2.getYaw().setUpdateFrequency(100);
        // _pigeon2.getPitch().setUpdateFrequency(100);
        _pigeon.reset();
        _pigeon.setYaw(0);
        
        _pigeon.getYaw();
        _pigeon.getPitch();
        // _pigeon2.setStatusFramePeriod(0,100 )
        // _pigeon2.getGravityVectorZ().setUpdateFrequency(100);
    }

    public void reset() {
        initPigeon();
    }

    public double getYaw() {
        return _pigeon.getYaw();
    }

    public double getPitch() {
        return _pigeon.getPitch();
    }

    public double getRoll() {
        return _pigeon.getRoll();
    }

    public Rotation2d getRotation2d() {
        return _pigeon.getRotation2d();
    }

    public void setYaw(double yaw) {
        _pigeon.setYaw(yaw, 10);
    }

    public void addYaw(double yaw) {
        _pigeon.addYaw(yaw, 10);
    }

    public void setYawToCompass() {
        _pigeon.setYawToCompass(10);
    }

    public void setAccumZ(double accumZ) {
        _pigeon.setAccumZAngle(accumZ, 10);
    }

    public Pigeon2_Faults getFaults() {
        return _pigeonFaults;
    }

    public boolean getFault() {
        return _pigeonFaults.hasAnyFault();
    }

    public double getCompass() {
        return _pigeon.getCompassHeading();
    }

    public double getAccumZ() {
        double[] accums = new double[3];
        _pigeon.getAccumGyro(accums);
        return accums[2];
    }

    public double[] getRawGyros() {
        double[] gyrs = new double[3];
        _pigeon.getRawGyro(gyrs);
        return gyrs;
    }

    public int getUpTime() {
        return _pigeon.getUpTime();
    }

    public double getTemp() {
        return _pigeon.getTemp();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        _pigeon.getFaults(_pigeonFaults);
        SmartDashboard.putNumber("PIGEON_PITCH", getPitch());
        SmartDashboard.putNumber("PIGEON_TEMP", getTemp());
        SmartDashboard.putNumber("PIGEON_ANGLE", getAngle());
        SmartDashboard.putNumber("PIGEON_RATE", getRate());
        SmartDashboard.putNumber("PIGEON_YAW", getYaw());
    }

    public String getFaultMessage() {
        if (!_pigeonFaults.hasAnyFault())
            return "No faults";
        String retval = "";
        retval += _pigeonFaults.APIError ? "APIError, " : "";
        retval += _pigeonFaults.AccelFault ? "AccelFault, " : "";
        retval += _pigeonFaults.BootIntoMotion ? "BootIntoMotion, " : "";
        retval += _pigeonFaults.GyroFault ? "GyroFault, " : "";
        retval += _pigeonFaults.HardwareFault ? "HardwareFault, " : "";
        retval += _pigeonFaults.MagnetometerFault ? "MagnetometerFault, " : "";
        retval += _pigeonFaults.ResetDuringEn ? "ResetDuringEn, " : "";
        retval += _pigeonFaults.SaturatedAccel ? "SaturatedAccel, " : "";
        retval += _pigeonFaults.SaturatedMag ? "SaturatedMag, " : "";
        retval += _pigeonFaults.SaturatedRotVelocity ? "SaturatedRotVelocity, " : "";
        return retval;
    }

    public double getAngle() {
        return _pigeon.getAngle();
    }

    public double getRate() {
        return _pigeon.getRate();
    }

    // public abstract String getFaultMessage();
}