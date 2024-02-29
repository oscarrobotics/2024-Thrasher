package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    //2 NEO Vortex Flexes for Shooter
    //Absolute Encoder
    AnalogPotentiometer m_pivPotentiometer;

    private DigitalInput m_shootBeamBreaker;

    private CANSparkFlex m_leftShootMotor, m_rightShootMotor;
    private TalonFX m_shootPivotMotor, m_sledPivotMotor;

    private DutyCycleEncoder m_absoluteEncoder;

    private boolean isStowed;

    TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(1,1);
    ProfiledPIDController m_controller = 
        new ProfiledPIDController(0, 0, 0, m_constraints, 0.02); 
    
    DutyCycleOut motorRequest = new DutyCycleOut(0.0); 
    double position = m_absoluteEncoder.getAbsolutePosition();

    public Shooter(){
        m_leftShootMotor = new CANSparkFlex(0, MotorType.kBrushless);
        m_rightShootMotor = new CANSparkFlex(1, MotorType.kBrushless);

        m_sledPivotMotor = new TalonFX(1);
        m_shootPivotMotor = new TalonFX(2, "rio");

        m_absoluteEncoder = new DutyCycleEncoder(0);

        m_pivPotentiometer = new AnalogPotentiometer(1);
        m_shootBeamBreaker = new DigitalInput(3);
    }

    //debugging --> encoder output, control the motor, PID values

    /* States */
    public boolean isInSled(){
        isStowed = (!m_shootBeamBreaker.get())?true:false;
        return isStowed;
    }

    /* Velocity */
      public Command toWheelSpeeds(double velocity){
        // frontWheelTargetSpeed = velocity.in(Rotations.per(Minute));
        return runEnd(() -> {
            m_leftShootMotor.set(velocity);
            m_rightShootMotor.set(velocity);
        }, () -> {
            m_leftShootMotor.set(0);
            m_leftShootMotor.set(0);
        });
    }
    /* Pivot */
    public double getPivotAbsPosition(){
        return m_absoluteEncoder.getAbsolutePosition();
    }

    public double getPivotAngle(){
        return m_absoluteEncoder.get();
    }

    public void setTargetAngle(double angle){
        m_controller.setGoal(angle);
    }

    /* Commands */
    //gotta think on what to do here!
    
    @Override
    public void periodic(){
        m_shootPivotMotor.setControl(motorRequest.withOutput(m_controller.calculate(m_absoluteEncoder.get())));
    }
}
