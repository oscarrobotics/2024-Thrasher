package frc.robot.Commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command; //CommandBase is deprecated to Command
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Sled;
import frc.robot.Subsystems.Shooter;

public class Ampfire extends Command {
    private Shooter m_shooter;
    private Sled m_sled;
    private double runtime;
    private boolean isFirstExecute;
    private Timer m_timer = new Timer();
 


    public Ampfire(Shooter shooter, Sled sled){
        this.m_shooter = shooter;
        this.m_sled = sled;
        this.runtime = 2;
        addRequirements(shooter, sled);

        // parallel = new ParallelRaceGroup(m_intake.intakeCommand(), m_sled.feed());

       
    }
    public Ampfire(Shooter shooter, Sled sled, double time){
        this.m_shooter = shooter;
        this.m_sled = sled;
        this.runtime = time;
        addRequirements(shooter, sled);
        isFirstExecute = true;


    }

  

    @Override
    public void initialize(){  
        isFirstExecute = true;
        m_shooter.tilt_amp();
        // m_sled.tilt_amp();
        m_timer.reset();
        m_timer.start();
      
    }
    @Override
    public void execute(){
        if(m_shooter.isShootAligned()){
            if(isFirstExecute){
            m_timer.reset();
            isFirstExecute = false;
            }
            m_shooter.unload_amp();
        }
        if(isFirstExecute){
            m_timer.reset();
        }
        
        
    }

    @Override
    public boolean isFinished(){
        //if exceeds delta t: stop
        return m_timer.hasElapsed(runtime)  ;  
     
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.stop();
        m_sled.stop();
        // m_sled.goToSledPose();
        m_timer.stop();
        isFirstExecute = true;
   
    }
}
