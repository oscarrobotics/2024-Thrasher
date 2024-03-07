package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class Shoot_note extends Command{
    private Shooter m_shooter;
    private Timer m_timer;
    public Shoot_note(Shooter m_shooter){
        this.m_shooter = m_shooter;
        this.m_timer = m_timer;
    }
}
