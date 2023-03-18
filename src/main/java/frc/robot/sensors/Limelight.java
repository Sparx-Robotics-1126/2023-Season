package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.LimelightConstants.*;

public class Limelight {
	NetworkTableEntry tx;
	NetworkTableEntry tv;
	NetworkTableEntry ty;

	NetworkTableEntry thor;
	NetworkTableEntry tvert;

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

		// The horizontal viewport size.
		thor = table.getEntry("thor");

		// The vertical viewport size.
		tvert = table.getEntry("tver");

		ledMode = table.getEntry("ledMode");
		camMode = table.getEntry("camMode");

		setForTargeting(USE_FOR_TARGETING);
		setLED(LED_ON_DEFAULT);
	}

	public double getCameraHeight() {
		// Add any necessary offsets here.
		return CAMERA_MIN_FLOOR_HEIGHT;
	}

	public double getCameraAngle() {
		// Add any necessary offsets here.
		return CAMERA_ANGLE;
	}

	public long getViewportXSize() {
		return thor.getInteger(0);
	}

	public long getViewportYSize() {
		return tvert.getInteger(0);
	}
	
	public double getDistanceFromTarget(double targetFloorHeight) {
		return (targetFloorHeight - getCameraHeight())
			/ Math.tan(Math.toRadians(getCameraAngle() + getTargetY()));
	}

	public long getTargetX() {
		return tx.getInteger(0);
	}

	public long getTargetY() {
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