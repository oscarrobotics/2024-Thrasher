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

    //basis command
    // new InstantCommand(()->m_shooter.tilt_strait()),
    // new WaitCommand(0.4).until(()->m_shooter.isShootAligned()),
    // new InstantCommand( () -> m_shooter.shootNote(), m_shooter),
    // new WaitCommand(0.5),
    // new InstantCommand( () ->m_sled.runSled(), m_sled),
    // new WaitCommand(1),
    // new ParallelCommandGroup(
    // new InstantCommand(() -> m_shooter.stop(), m_shooter),
    // new InstantCommand(() -> m_sled.stop(), m_sled))

    public Shoot_note(Shooter m_shooter, Sled m_sled){
        this.m_shooter = m_shooter;
        this.m_sled = m_sled;
        m_timer = new Timer();
        runtime = 3;
        addRequirements(m_shooter, m_sled);
    }

    @Override
    public void initialize(){
        // m_shooter.setTargetTilt(10);
        m_timer.reset();
        m_timer.start();
        m_shooter.tilt_strait();

        isFirstExecute = true;
    }

    @Override
    public void execute(){
    if(m_shooter.isShootAligned()){
        if(isFirstExecute){
            m_timer.reset();
            isFirstExecute = false;
        }
        m_shooter.shootNote();
        if(m_timer.hasElapsed(0.5)){
            m_sled.runSled();
            // m_timer.reset();
        }
    }
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return m_timer.hasElapsed(1.5);
    }
    
    @Override
    public void end(boolean interrupted){
        isFirstExecute = true;
        m_timer.stop();
        m_shooter.stop();
        m_sled.stop();
    }
}
