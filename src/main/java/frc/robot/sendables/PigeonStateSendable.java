// package frc.robot.sendables;
// import com.ctre.phoenix.sensors.WPI_Pigeon2;

// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;


// public class PigeonStateSendable implements Sendable {
//     private WPI_Pigeon2 m_pigeon;

//     public PigeonStateSendable(WPI_Pigeon2 pigeon) {
//         m_pigeon = pigeon;
//     }

//     public void initSendable(SendableBuilder builder) {
//         builder.setSmartDashboardType("PigeonState");
//         builder.addDoubleProperty("yaw", m_pigeon::getYaw, m_pigeon::setYaw);
//         builder.addDoubleProperty("pitch", m_pigeon::getPitch, null);
//         builder.addDoubleProperty("roll", m_pigeon::getRoll, null);
//         builder.addBooleanProperty("faults", m_pigeon::getFault, null);
//         builder.addStringProperty("fault message", m_pigeon::getFaultMessage, null);
//         builder.addDoubleProperty("Compass Heading", m_pigeon::getCompass, null);
//         builder.addDoubleProperty("AccumZ", m_pigeon::getAccumZ, null);
//         builder.addDoubleArrayProperty("Raw Gyros", m_pigeon::getRawGyros, null);
//         builder.addDoubleProperty("Up Time", m_pigeon::getUpTime, null);
//         builder.addDoubleProperty("Temperature", m_pigeon::getTemp, null);
//     }
// }
