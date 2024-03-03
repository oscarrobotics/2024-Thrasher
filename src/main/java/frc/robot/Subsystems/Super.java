package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Super extends SubsystemBase{
    
    protected final Intake m_intake;
    protected final  Shooter m_shooter;

    private TalonFX m_sledMotor;
    private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);

    public Super(Intake intake, Shooter shooter){
        this.m_intake = intake;
        this.m_shooter = shooter;

        m_sledMotor = new TalonFX(5);
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

        return Commands.parallel(intake, feed);
    }

    public Command shoot(){
        Command shoot = m_shooter.shootNote();
        Command feed = runSled();
        
        return Commands.parallel(shoot, feed);
    }
}
