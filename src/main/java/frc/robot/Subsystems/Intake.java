package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    //2 Falcons --> Intake
    public TalonFX m_frontIntakeMotor;
    public TalonFX m_rearIntakeMotor;
    public DigitalInput m_intakeBeamBreaker;
    private boolean isStowed;

    public Intake(){
        m_intakeBeamBreaker = new DigitalInput(0);
    }

       /* SLED */
    public boolean isInSled(){
        isStowed = (!m_intakeBeamBreaker.get())?true:false;
        return isStowed;
    }

    // ????????? Genuinely the worst thing I've ever seen
    // public Command stowedInSled(Supplier<Boolean> isStowed){
    //     return runOnce(() -> {isStowed.equals(isInSled());});
    // }

    public Command intake(){
        BooleanSupplier weAreStowed = () -> isInSled();
        return run(null).until(weAreStowed);}
    }
    //Intake piece cmd
    //Reject piece cmd

