package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * The ideal voltage that the motors should attempt to match.
     */
    //
    public static final double NOMINAL_VOLTAGE = 12;
    public static final int MAX_CURRENT = 25;
    public static final double DEAD_BAND = .5;
    public static final int Pigeon2ID = 4;
    public static final int XBOX_DRIVER_CONTROLLER_PORT = 0;
    public static final int XBOX_OPERATOR_CONTROLLER_PORT = 1;

    public static final class DriveConstants {
        // MOTORS
        public static final int DRIVES_RIGHT_MOTOR_1 = 25;
        public static final int DRIVES_RIGHT_MOTOR_2 = 27;
        public static final int DRIVES_LEFT_MOTOR_1 = 24;
        public static final int DRIVES_LEFT_MOTOR_2 = 26;

        public static final double MAX_DRIVE_SPEED = 0.8;
        public static final double MAX_TRIGGER_SPEED = 0.1;
        public static final double kTurnFriction = 0.3;

        public static final int kEncoderCPR = 1024;
        public static final double kGearRatio = 10.75; //8.45;// 10.71;
        public static final double kWheelDiameterInches = 6;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kEncoderDistanceConversionFactor = ( (Math.PI * kWheelDiameterMeters) / (kGearRatio));

        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;

        public static final double kTurnP = .00422125;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        public static final double kMaxTurnRateDegPerS = 100;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;

        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 5; // degrees per second

        public static final double kAutoDriveForwardSpeed = 0.15;


        public static final double ENCODER_MULTIPLIER = 1.105;

        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;


        public static final double ksVolts = 0.198;   //0.169
        public static final double kvVoltSecondsPerMeter = 2.86;  //2.24
        public static final double kaVoltSecondsSquaredPerMeter = 0.365;  //0.0435
        public static final double kPDriveVel = 2.24;  //2.4

        public static final boolean kGyroReversed = false;
        
        public static final int EndGameSeconds = 105;
        public static final double kRelTurnP = .001;
        public static final double kRelTurnI = 0;
        public static final double kRelTurnD = 0;
        public static final double kRelTurnFriction = 0.8;
        
        public static final double kMaxRelTurnRateDegPerS = 20;
        public static final double kMaxRelTurnAccelerationDegPerSSquared = 300;

        public static final double kRelTurnToleranceDeg = 1;
        public static final double kRelTurnRateToleranceDegPerS = 3; // degrees per second

    }

    public static final class AcquisitionConstants {
        public static final int X_MOTOR = 9;
        public static final int Y_LEFT_MOTOR = 10;
        public static final int Y_RIGHT_MOTOR = 7;
        
        public static final int COMPRESSOR = 0;
        public static final int SOLENOID = 0;

        public static final int X_LIMIT = 9;
        public static final int Y_LIMIT = 8;

        public static final int X_ENCODER_A = 23;
        public static final int X_ENCODER_B = 22;
        public static final int Y_LEFT_ENCODER_A = 13;
        public static final int Y_LEFT_ENCODER_B = 12;
        public static final int Y_RIGHT_ENCODER_A = 25;
        public static final int Y_RIGHT_ENCODER_B = 24;

        public static final double MOTOR_P = 1.5;
        public static final double MOTOR_I = 0;
        public static final double MOTOR_D = 0;

        public static final double X_MAX = 1.1938;
        public static final double Y_MAX = 0.69215;
        public static final double POSITION_EPSILON_METERS = 0.01;
        
        public static final double RETURN_HOME_POWER = 0.5;
        public static final double MAX_MOTOR_POWER = 1;
        public static final double Y_FEEDFORWARD = 1;
        
        // TODO: remeasure these values.
        public static final double X_PULSES_TO_METERS = 1.0 / 734.5;
        public static final double Y_PULSES_TO_METERS = 1.0 / 1;
    }

    /**
     * All values are in meters.
     */
    public static final class FieldConstants {
        private static final double computeXOffset(double distance)
        {
            return distance + MIN_EXTENSION_FRONT + X_OFFSET;
        }

        private static final double computeYOffset(double height)
        {
            return height - REACH_MOUNT_POINT_HEIGHT + Y_OFFSET;
        }

        private static final double SHELF_HEIGHT = 0.95;
        private static final double SHELF_DISTANCE = 0.165;

        private static final double HIGH_CONE_HEIGHT = 1.17;
        private static final double HIGH_CONE_DISTANCE = 1.01;

        private static final double MID_CONE_HEIGHT = 0.87;
        private static final double MID_CONE_DISTANCE = 0.58;

        private static final double HIGH_CUBE_HEIGHT = 0.9;
        private static final double HIGH_CUBE_DISTANCE = 1.01;

        private static final double MID_CUBE_HEIGHT = 0.6;
        private static final double MID_CUBE_DISTANCE = 0.58;

        /**
         * The distance from the floor to the mount point of reach.
         */
        private static final double REACH_MOUNT_POINT_HEIGHT = 0.6858;
        /**
         * The distance from the minimum extension of reach to the front of the robot.
         */
        private static final double MIN_EXTENSION_FRONT = 0;

        /**
         * The distance added to all y values.
         */
        private static final double Y_OFFSET = 0.1;
        /**
         * The distance added to all x values.
         */
        private static final double X_OFFSET = 0.1;

        public static final double SHELF_Y = computeYOffset(SHELF_HEIGHT);
        public static final double SHELF_X = computeXOffset(SHELF_DISTANCE);

        public static final double HIGH_CONE_Y = computeYOffset(HIGH_CONE_HEIGHT);
        public static final double HIGH_CONE_X = computeXOffset(HIGH_CONE_DISTANCE);

        public static final double MID_CONE_Y = computeYOffset(MID_CONE_HEIGHT);
        public static final double MID_CONE_X = computeXOffset(MID_CONE_DISTANCE);

        public static final double HIGH_CUBE_Y = computeYOffset(HIGH_CUBE_HEIGHT);
        public static final double HIGH_CUBE_X = computeXOffset(HIGH_CUBE_DISTANCE);

        public static final double MID_CUBE_Y = computeYOffset(MID_CUBE_HEIGHT);
        public static final double MID_CUBE_X = computeXOffset(MID_CUBE_DISTANCE);
    }

    public static final class ChooserOptions{
        static final String kAutoShort = "Short";
         static final String kAutoLong = "Long";
         static final String kDriveMeasure = "Drive Measure";
         static final String kScoreCommunity = "Score and Leave Community";
    }

    public static final class AutoConstants {

        public static final int kPitchTimeoutSeconds = 5;
    }
}