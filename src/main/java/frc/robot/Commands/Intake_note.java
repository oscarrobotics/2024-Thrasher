package frc.robot.Commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command; //CommandBase is deprecated to Command
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Sled;

public class Intake_note extends Command {
    private Intake m_intake;
    private Sled m_sled;
    private double runtime;
    private Timer m_timer = new Timer();

    
 



    public Intake_note(Intake intake, Sled sled){
        this.m_intake = intake;
        this.m_sled = sled;
        this.runtime = 5;
        
        

        


       
    }
    public Intake_note(Intake intake, Sled sled, double time){
        this.m_intake = intake;
        this.m_sled = sled;
        this.runtime = time;
    }

  

    @Override
    public void initialize(){  
        m_sled.goToIntakePose();
        m_timer.start();
    
        
      
    }
    @Override
    public void execute(){
        m_intake.intake();
    }

    @Override
    public boolean isFinished(){
        //if exceeds delta t: stop
        return m_timer.hasElapsed(runtime) || m_sled.isInSled();  
     
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stop();
        // m_sled.goToSledPose();
        m_timer.stop();
   
    }
}
