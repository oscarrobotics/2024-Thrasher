package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Sled;

public class Shoot_note extends Command{
    private Shooter m_shooter;
    private Sled m_sled;
    private Timer m_timer;
    private double runtime;
    private boolean isFirstExecute;

    public Shoot_note(Shooter m_shooter, Sled m_sled){
        this.m_shooter = m_shooter;
        this.m_sled = m_sled;
        m_timer = new Timer();
        runtime = 3;
    }

    @Override
    public void initialize(){
        // m_shooter.setTargetTilt(10);
        m_timer.start();

        isFirstExecute = true;
    }

    @Override
    public void execute(){
        if(isFirstExecute){
            m_timer.reset();
            isFirstExecute = false;
        }
        m_shooter.shootNote();
        new WaitCommand(2);
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
