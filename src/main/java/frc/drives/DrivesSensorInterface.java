package frc.drives;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.RelativeEncoder;

/**
 * Any and all sensor data required for operation of the Drives subsystem such as encoders, limit switches, etc..
 */
public interface DrivesSensorInterface 
{
	void addEncoders(RelativeEncoder leftSpark, RelativeEncoder rightSpark);
    void addGyro(WPI_Pigeon2 pigeon);
	
    //Sensors.
    double getLeftEncoderDistance();
    double getLeftEncoderSpeed();
    double getRightEncoderDistance();
    double getRightEncoderSpeed();
    double getAverageEncoderDistance();
    double getAverageEncoderSpeed();

    //Operator input.
    double getRightJoyStick();
    double getLeftJoyStick();
    void setRightJoystick(double value);
    void setLeftJoystick(double value);
}