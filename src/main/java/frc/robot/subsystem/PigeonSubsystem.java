package frc.robot.subsystem;
// package frc.subsystem;

// import com.ctre.phoenix.sensors.WPI_Pigeon2;
// import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
// import com.ctre.phoenix.sensors.Pigeon2_Faults;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.wpiutils.*;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
// import com.ctre.phoenixpro.hardware.Pigeon2;

// import com.ctre.phoenix.sensors.Pigeon2Configuration;

public class PigeonSubsystem extends SubsystemBase {
    private final Pigeon2 _pigeon;

    // private Pigeon2 _test;
    // private static Pigeon2_Faults _pigeonFaults = new Pigeon2_Faults();
    // private BasePigeon m_basePigeon;

    public PigeonSubsystem() {
        _pigeon = new Pigeon2(Constants.Pigeon2ID);
        // _pigeonFaults = new Pigeon2_Faults();
        // _test = new Pigeon2(4);
        initPigeon();
    }

    private void initPigeon() {
        // Factory default the Pigeon.
        var toApply = new Pigeon2Configuration();
        //var mountPose = toApply.MountPose;

    /* User can change the configs if they want, or leave it empty for factory-default */
   
        _pigeon.getConfigurator().apply(toApply);

        // Add runtime adjustments to Pigeon configuration below this line.
       // _pigeon.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);

        // used by pro still thinking about it
        // _pigeon2.getConfigurator().apply(toApply);

        /* Speed up signals to an appropriate rate */
        // _pigeon2.getYaw().setUpdateFrequency(100);
        // _pigeon2.getPitch().setUpdateFrequency(100);
        _pigeon.reset();
        
        _pigeon.getYaw().setUpdateFrequency(100);
        _pigeon.getPitch().setUpdateFrequency(100);
        // _pigeon2.setStatusFramePeriod(0,100 )
        // _pigeon2.getGravityVectorZ().setUpdateFrequency(100);
    }

    public void reset() {
        initPigeon();
    }

    
    /** 
     * @return double
     */
    public double getYaw() {
        return _pigeon.getYaw().getValue();
    }

    
    /** 
     * @return The current pitch reported by the Pigeon.
     */
    public int getPitch() {
        return (int) -Math.round(_pigeon.getPitch().getValue());
    }

    
    /** 
     * @return double
     */
    public double getRoll() {
        return _pigeon.getRoll().getValue();
    }

    
    /** 
     * @return Rotation2d
     */
    public Rotation2d getRotation2d() {
        return _pigeon.getRotation2d();
    }

    
    /** 
     * @param yaw
     */
    public void setYaw(double yaw) {
        _pigeon.setYaw(yaw, 10);
    }

    
    /** 
     * @param yaw
     */
    // public void addYaw(double yaw) {
    //     _pigeon.addYaw(yaw, 10);
    // }

    // public void setYawToCompass() {
    //     _pigeon.setYawToCompass(10);
    // }

    
    /** 
     * @param accumZ
     */
    // public void setAccumZ(double accumZ) {
    //     _pigeon.setAccumZAngle(accumZ, 10);
    // }

    
    /** 
     * @return Pigeon2_Faults
     */
    // public Pigeon2_Faults getFaults() {
    //     return _pigeon.getf;
    // }

    
    /** 
     * @return boolean
     */
    // public boolean getFault() {
    //     return _pigeonFaults.hasAnyFault();
    // }

    
    /** 
     * @return double
     */
    // public double getCompass() {
    //     return _pigeon.getCompassHeading();
    // }

    
    /** 
     * @return double
     */
    // public double getAccumZ() {
    //     double[] accums = new double[3];
    //     _pigeon.getAccumGyroZ(accums);
    //     return accums[2];
    // }

    
    /** 
     * @return double[]
     */
    // public double[] getRawGyros() {
    //     double[] gyrs = new double[3];
    //     _pigeon.getRawGyro(gyrs);
    //     return gyrs;
    // }

    
    /** 
     * @return int
     */
    public double getUpTime() {
        return _pigeon.getUpTime().getValue();
    }

    
    /** 
     * @return double
     */
    public double getTemp() {
        return _pigeon.getTemperature().getValue();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // _pigeon.getFaults(_pigeonFaults);
        SmartDashboard.putNumber("PIGEON_PITCH", getPitch());
        SmartDashboard.putNumber("PIGEON_TEMP", getTemp());
        SmartDashboard.putNumber("PIGEON_ANGLE", getAngle());
        SmartDashboard.putNumber("PIGEON_RATE", getRate());
        SmartDashboard.putNumber("PIGEON_YAW", getYaw());
        SmartDashboard.putNumber("PIGEON_ROTATION", getRotation2d().getDegrees());
    }

    
    /** 
     * @return String
     */
    // public String getFaultMessage() {
    //     if (!_pigeonFaults.hasAnyFault())
    //         return "No faults";
    //     String retval = "";
    //     retval += _pigeonFaults.APIError ? "APIError, " : "";
    //     retval += _pigeonFaults.AccelFault ? "AccelFault, " : "";
    //     retval += _pigeonFaults.BootIntoMotion ? "BootIntoMotion, " : "";
    //     retval += _pigeonFaults.GyroFault ? "GyroFault, " : "";
    //     retval += _pigeonFaults.HardwareFault ? "HardwareFault, " : "";
    //     retval += _pigeonFaults.MagnetometerFault ? "MagnetometerFault, " : "";
    //     retval += _pigeonFaults.ResetDuringEn ? "ResetDuringEn, " : "";
    //     retval += _pigeonFaults.SaturatedAccel ? "SaturatedAccel, " : "";
    //     retval += _pigeonFaults.SaturatedMag ? "SaturatedMag, " : "";
    //     retval += _pigeonFaults.SaturatedRotVelocity ? "SaturatedRotVelocity, " : "";
    //     return retval;
    // }

    
    /** 
     * @return double
     */
    public double getAngle() {
        return _pigeon.getAngle();
    }

    
    /** 
     * @return double
     */
    public double getRate() {
        return _pigeon.getRate();
    }

    // public abstract String getFaultMessage();
}