package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.LimelightConstants.*;

public class Limelight {
	NetworkTableEntry tx;
	NetworkTableEntry tv;
	NetworkTableEntry ty;

	NetworkTableEntry ledMode;
	NetworkTableEntry camMode;
	
	public Limelight() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

		// 1 if a target is present, or 0 if not.
		tv = table.getEntry("tv");

		// The X offset of the target in degrees from the crosshair.
		tx = table.getEntry("tx");

		// The Y offset of the target in degrees from the crosshair.
		ty = table.getEntry("ty");

		ledMode = table.getEntry("ledMode");
		camMode = table.getEntry("camMode");

		setForTargeting(USE_FOR_TARGETING);
		setLED(LED_ON_DEFAULT);
	}

	public double getCameraHeight() {
		// Add any necessary robot-specific dynamic offsets here (e.g. system elevates the limelight).
		return CAMERA_MIN_FLOOR_HEIGHT;
	}

	public double getCameraPitch() {
		// Add any necessary robot-specific dynamic offsets here (e.g. system tilts the limelight up and down).
		return CAMERA_INITIAL_PITCH;
	}

	public double getXAngle() {
		return getXCrosshairAngle();
	}

	public double getYAngle() {
		return getCameraPitch() + getYCrosshairAngle();
	}

	public double getYOffset(double targetFloorHeight) {
		return targetFloorHeight - getCameraHeight();
	}
	
	public double getHorizontalDistance(double targetFloorHeight) {
		return getDistance(targetFloorHeight) 
			* Math.tan(Math.toRadians(getXAngle()));
	}

	public double getDistance(double targetFloorHeight) {
		return getYOffset(targetFloorHeight)
			/ Math.tan(Math.toRadians(getYAngle()));
	}

	public long getXCrosshairAngle() {
		return tx.getInteger(0);
	}

	public long getYCrosshairAngle() {
		return ty.getInteger(0);
	}
	
	public boolean hasLock() {
		return tv.getInteger(0) > 0;
	}

	public void setForTargeting(boolean enable) {
		int camModeNum = enable ? 0 : 1;
		camMode.setNumber(camModeNum);
	}
	
	public void setLED(boolean enable) {
		int ledModeNum = enable ? 3 : 1;
		ledMode.setNumber(ledModeNum);
	}
}