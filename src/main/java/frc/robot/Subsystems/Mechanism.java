package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Mechanism extends SubsystemBase{
    
    DoublePublisher T_sledPivot;
    DoublePublisher T_shooterPivot;

    BooleanPublisher T_sledBreak;
    BooleanPublisher T_inSled;
    BooleanPublisher T_shootBreak;



    public final Intake m_intake;
    public final  Shooter m_shooter;

    private TalonFX m_sledMotor;
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    




    public Mechanism(){
    
        m_intake = new Intake();
        m_shooter = new Shooter();

        
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable NT = inst.getTable("Mechanism");
        
        T_sledPivot = NT.getDoubleTopic("sledPivot").publish();
        T_shooterPivot = NT.getDoubleTopic("shootPivot").publish();

        T_sledBreak = NT.getBooleanTopic("SledBreak").publish();
        T_shootBreak = NT.getBooleanTopic("shootBreak").publish();
        T_inSled = NT.getBooleanTopic("inSled").publish();

        m_sledMotor = new TalonFX(Constants.kSledIntakeId);

        TalonFXConfiguration config = new TalonFXConfiguration();  

        
        
        config.Slot0.kP = 0.125;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        config.Voltage.PeakForwardVoltage = 8;
        config.Voltage.PeakReverseVoltage = -8; 

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_sledMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
          }
          if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
          }
          

       
        

        


    }
    
    // public Command toSledSpeeds(){
    //     // var velocity = vel.get();   
        
    // }
    // final Supplier<Measure<Velocity<Angle>>> speed = () -> Rotations.per(Minute).of(0);
    // <Measure<Velocity<Angle>>> speed = () -> Rotations.per(Minute).of(0);
    public Command runSled(){
        // return run( () -> { toSledSpeeds( () -> Rotations.per(Minute).of(0)); } );
        return runEnd(() -> { 
            // m_sledMotor.setControl(m_request.withVelocity(velocity.in(Rotations.per(Minute))));
            // m_sledMotor.setControl(m_request.withVelocity(speed.in(Rotations.per(Minute))));
            m_sledMotor.setControl(m_request.withVelocity(-90));
        }, 
        () -> { m_sledMotor.setControl(m_request.withVelocity(0));});
        // return run(()->{System.out.println("sled");}).withTimeout(0.1);
    }

    public Command intake(){
        Command intake = m_intake.intake();
        Command feed = runSled();
        

        return Commands.race(intake,feed);
        // return intake;
        // return feed;
        
        // return run(()->{System.out.println("intake");}).withTimeout(0.1);
    }

    public Command outtake(){
        Command intake = m_intake.reject();
        Command feed = runEnd(() -> { 
            m_sledMotor.setControl(m_request.withVelocity(90)); //Opposite sign
        }, 
        () -> { m_sledMotor.setControl(m_request.withVelocity(0));});

        return Commands.race(intake, feed);
    }

    public Command tilt_down(){
        if (m_shooter.getSledPivotAngle()<=50) {
            return runOnce(()->{m_shooter.setTargetSledPivot(m_shooter.getSledPivotAngle()+5);}).withTimeout(1);
            
        }
        return runOnce(()->{m_shooter.setTargetSledPivot(55);}).withTimeout(1);
        
    }

    public Command tilt_up(){
        if (m_shooter.getSledPivotAngle()>=5) {
            return runOnce(()->{m_shooter.setTargetSledPivot(m_shooter.getSledPivotAngle()-5);}).withTimeout(1);
            
        }
        return runOnce(()->{m_shooter.setTargetSledPivot(0);}).withTimeout(1);
        
    }

    public Command tilt_to(double angle){
        return runOnce(()->{m_shooter.setTargetSledPivot(angle);}).withTimeout(1);
    }

    //TODO: a way to zero out the pivot angle. Does it need to start at the same angle every time?
    // public Command reset_pivot(){
    //     return runOnce(()->{m_shooter.setTargetSledPivot(0);}).withTimeout(0);
    // }


    //feed note into shoot cmd

    public Command shoot(){
        Command shoot = m_shooter.shootNote();
        Command feed = runSled();

        return Commands.parallel(shoot, feed);
    }

    public Command debug_runner(Supplier<Double> sledangle){
        return runEnd(() -> {
            m_shooter.setTargetSledPivot(sledangle.get());
        }, () -> {
            // m_sledMotor.setControl(m_request.withVelocity(0));
        });
    }   
    @Override
        public void periodic() {
      
        T_shootBreak.set(m_shooter.get_beam());
        T_sledBreak.set(m_intake.get_beam());

        T_shooterPivot.set(m_shooter.getShootPivotAngle());
        T_sledPivot.set(m_shooter.getSledPivotAngle());
        
        T_inSled.set(m_intake.isInSled());  

        

    }
}
