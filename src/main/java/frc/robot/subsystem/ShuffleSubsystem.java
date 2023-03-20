package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ShuffleSubsystem extends SubsystemBase{
   
    public abstract void update();
    public abstract void displayShuffleboard();
    public abstract void tuningInit();
    public abstract void tuningPeriodic();
}
