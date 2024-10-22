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

public class Unjam_note extends Command {
    private Intake m_intake;
    private Sled m_sled;
    private double runtime;
    private boolean isFirstExecute;
    private Timer m_timer = new Timer();
 
    private ParallelRaceGroup parallel;

    public Unjam_note(Intake intake, Sled sled){
        this.m_intake = intake;
        this.m_sled = sled;
        this.runtime = 0.1;
        addRequirements(intake, sled);
        // parallel = new ParallelRaceGroup(m_intake.intakeCommand(), m_sled.feed());

       
    }
    public Unjam_note(Intake intake, Sled sled, double time){
        this.m_intake = intake;
        this.m_sled = sled;
        this.runtime = time;
        addRequirements(intake, sled);

        isFirstExecute = true;


    }

  

    @Override
    public void initialize(){  
        isFirstExecute = true;
        m_sled.goToIntakePose();
        m_timer.reset();
        m_timer.start();
      
    }
    @Override
    public void execute(){
        if(isFirstExecute){
            m_timer.reset();
            isFirstExecute = false;
        }
        m_intake.outtake();
        m_sled.unrunSled();
    }

    @Override
    public boolean isFinished(){
        //if exceeds delta t: stop
        return m_timer.hasElapsed(runtime);
        // || m_sled.isInSled();  
     
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stop();
        m_sled.stop();
        // m_sled.goToSledPose();
        m_timer.stop();
        isFirstExecute = true;
   
    }
}
