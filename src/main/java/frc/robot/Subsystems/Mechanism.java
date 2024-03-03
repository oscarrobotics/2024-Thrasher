package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
    BooleanPublisher T_shootBreak;






    
    protected final Intake m_intake;
    protected final  Shooter m_shooter;

    private TalonFX m_sledMotor;
    private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);

    public Mechanism(){
    
        m_intake = new Intake();
        m_shooter = new Shooter();

        m_sledMotor = new TalonFX(Constants.kSledIntakeId);
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable NT = inst.getTable("Mechanism");
        
        T_sledPivot = NT.getDoubleTopic("sledPivot").publish();
        T_shooterPivot = NT.getDoubleTopic("shootPivot").publish();

        T_sledBreak = NT.getBooleanTopic("SledBreak").publish();
        T_shootBreak = NT.getBooleanTopic("shootBreak").publish();


    }

    public Command toSledSpeeds(Supplier<Measure<Velocity<Angle>>> vel){
        var velocity = vel.get();
        return runEnd(() -> { 
            m_sledMotor.setControl(m_request.withVelocity(velocity.in(Rotations.per(Minute))));
        }, 
        () -> { m_sledMotor.setControl(m_request.withVelocity(0));});
    }
    public Command runSled(){
        return runOnce( () -> { toSledSpeeds( () -> Rotations.per(Minute).of(3000)); } );
    }

    public Command intake(){
        Command intake = m_intake.intake();
        Command feed = runSled();
        

        return Commands.race(intake,feed);
    }

    //feed note into shoot cmd

    public Command shoot(){
        Command shoot = m_shooter.shootNote();
        Command feed = runSled();

        return Commands.parallel(shoot, feed);
    }

    @Override
        public void periodic() {
      
        T_shootBreak.set(m_shooter.get_beam());
        T_sledBreak.set(m_intake.get_beam());

        T_shooterPivot.set(m_shooter.getShootPivotAngle());
        T_sledPivot.set(m_shooter.getSledPivotAngle());

        

    }
}
