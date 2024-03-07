package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Sled;

public class Shoot_note extends Command{
    private Shooter m_shooter;
    private Sled m_sled;
    private Timer m_timer;
    private double runtime;
    public Shoot_note(Shooter m_shooter, Sled m_sled){
        this.m_shooter = m_shooter;
        this.m_sled = m_sled;
    }

    @Override
    public void initialize(){
        // m_shooter.setTargetTilt(10);
        m_timer.start();
    }

    @Override
    public void execute(){
        m_shooter.shootNote();
        m_sled.runSled();
    }

    @Override
    public boolean isFinished(){
        return m_timer.hasElapsed(runtime);
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.stop();
        m_sled.stop();
    }
}
