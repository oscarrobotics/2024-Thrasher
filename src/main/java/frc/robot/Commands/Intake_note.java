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
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Sled;

public class Intake_note extends Command {
    private Intake m_intake;
    private Sled m_sled;
    private double runtime;
    private boolean isFirstExecute;
    private boolean isFirstLoad;
    private Timer m_timer = new Timer();
    private Timer m_backtimer = new Timer();
    private Timer m_intimer = new Timer();
 
    private ParallelRaceGroup parallel;

    public Intake_note(Intake intake, Sled sled){
        this.m_intake = intake;
        this.m_sled = sled;
        this.runtime = 1.2;
        addRequirements(intake, sled);

        // parallel = new ParallelRaceGroup(m_intake.intakeCommand(), m_sled.feed());

       
    }
    public Intake_note(Intake intake, Sled sled, double time){
        this.m_intake = intake;
        this.m_sled = sled;
        this.runtime = time;
        addRequirements(intake, sled);
        isFirstExecute = true;
        isFirstLoad = true;
    


    }

  

    @Override
    public void initialize(){  
        isFirstExecute = true;
        m_sled.goToIntakePose();
        m_timer.reset();
        m_backtimer.reset();
        m_timer.start();
        isFirstLoad = true;
      
    }
    @Override
    public void execute(){
        if(isFirstExecute){
            m_timer.reset();
            m_intimer.restart();
            isFirstExecute = false;
        }
        if(!m_intimer.hasElapsed(1.2) && !m_sled.interruptRequest){
            m_intake.intake();
            m_sled.runSled();
            m_backtimer.restart();
            isFirstLoad = false;

            

        }
        if ((m_intimer.hasElapsed(1.2) || m_sled.interruptRequest)&&!m_backtimer.hasElapsed(0.1)) {

        m_sled.unrunSled();
            
        }

    }

    @Override
    public boolean isFinished(){
        //if exceeds delta t: stop
        return m_timer.hasElapsed(runtime) || m_backtimer.hasElapsed(0.3);  
     
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stop();
        m_sled.stop();
        // m_sled.goToSledPose();
        m_timer.stop();
        isFirstExecute= false;
   
    }
}
