package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

public class Intake extends SubsystemBase{
    //2 Falcons --> Intake
    public Timer m_timer;
    public TalonFX m_frontIntakeMotor, m_rearIntakeMotor, m_frontSledMotor, m_rearSledMotor;

    public DigitalInput m_intakeBeamBreaker;
    private boolean isStowed;
     private final VelocityVoltage m_request = new VelocityVoltage(0);
     double frontWheelTargetSpeed;

    public Intake(){
        m_intakeBeamBreaker = new DigitalInput(0);
        m_frontIntakeMotor = new TalonFX(3);
        m_rearIntakeMotor = new TalonFX(4);
        m_frontSledMotor = new TalonFX(5);
        m_rearSledMotor = new TalonFX(6);
    }

       /* SLED */
    public boolean isInSled(){
        isStowed = (!m_intakeBeamBreaker.get())?true:false;
        return isStowed;
    }

    // ????????? Genuinely the worst thing I've ever seen
    // public Command stowedInSled(Supplier<Boolean> isStowed){
    //     return runOnce(() -> {isStowed.equals(isIn
        // Sled());});
    // }

    public Command toWheelSpeeds(Supplier<Measure<Velocity<Angle>>> vel){
        // frontWheelTargetSpeed = velocity.in(Rotations.per(Minute));
        return runEnd(() -> {
            var velocity = vel.get();
            m_frontIntakeMotor.setControl(m_request.withVelocity(velocity.in(Rotations.per(Second))));
            m_rearIntakeMotor.setControl(m_request.withVelocity(velocity.in(Rotations.per(Second))));
            m_frontSledMotor.setControl(m_request.withVelocity(0.5 * velocity.in(Rotations.per(Second))));
            m_rearSledMotor.setControl(m_request.withVelocity(0.5 * velocity.in(Rotations.per(Second))));
        }, () -> {
            m_frontIntakeMotor.setControl(m_request.withVelocity(0));
            m_rearIntakeMotor.setControl(m_request.withVelocity(0));
            m_frontSledMotor.setControl(m_request.withVelocity(0));
            m_rearSledMotor.setControl(m_request.withVelocity(0));
        });
    }
    public Command intake(){
        BooleanSupplier weAreStowed = () -> isInSled();
        return runOnce(() -> {toWheelSpeeds(() -> Rotations.per(Minute).of(7000));}).until(weAreStowed);
    }

    public Command reject(){
        return runOnce(
            () -> {toWheelSpeeds(() -> Rotations.per(Minute).of(-7000))
            .until(() -> {
                return m_timer.hasElapsed(1);} //idk how much time
            );
        });
    }
    //Intake piece cmd
    //Reject piece cmd
}
