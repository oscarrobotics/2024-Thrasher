package frc.robot.Subsystems;

import frc.robot.Constants;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Notifier;


public class Shooter extends SubsystemBase{
    //2 NEO Vortex Flexes for Shooter
    //Absolute Encoder
    private Timer m_Timer;
 

    private DigitalInput m_shootBeamBreaker;
    // private final Notifier m_note_shot_timings;
    // private boolean m_lastbeam = true;

    // private final Notifier m_faster_tilt_pid;



    private CANSparkFlex m_leftShootMotor, m_rightShootMotor;
    private SparkPIDController m_leftPID, m_rightPID;
    private TalonFX  m_shootPivotMotor;

    private DutyCycleEncoder m_absoluteEncoder;

    private TrapezoidProfile.Constraints m_Tiltconstraints = new TrapezoidProfile.Constraints(40,120);

    private ProfiledPIDController m_Tiltcontroller = 
        new ProfiledPIDController(1.2, 0.01, 0.12, m_Tiltconstraints, 0.02);
         

    private final VelocityVoltage m_request = new VelocityVoltage(0);

    private DutyCycleOut motorRequest = new DutyCycleOut(0.0); 

    boolean isShootAligned;
    double targetAngle;

   DoublePublisher T_shootPivotControllerOutput;

    DoublePublisher T_shootPivot;

    BooleanPublisher T_shootBreak;

    double leftTargetSpd, rightTargetSpd, leftError, rightError;

    

    public Shooter(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable NT = inst.getTable("Shooter");
      
        T_shootPivot = NT.getDoubleTopic("shootPivot").publish();
   
        T_shootBreak = NT.getBooleanTopic("shootBreak").publish();
       
        T_shootPivotControllerOutput = NT.getDoubleTopic("pivOutput").publish();
        
        
        m_leftShootMotor = new CANSparkFlex(Constants.kLeftShootMotorId, MotorType.kBrushless);
        m_rightShootMotor = new CANSparkFlex(Constants.kRightShootMotorId, MotorType.kBrushless);

        m_leftPID = m_leftShootMotor.getPIDController();
        m_rightPID = m_rightShootMotor.getPIDController();

        m_leftPID.setP(0.01);
        m_leftPID.setI(0.000);
        m_leftPID.setD(0.000);
        m_leftPID.setFF(0.002);

        m_rightPID.setP(0.01);
        m_rightPID.setI(0.000);
        m_rightPID.setD(0.000);
        m_rightPID.setFF(0.002); //12V / 6000 RPM

        
        //sled pivot
        


        // m_sledPivotMotor = new TalonFX(1);
        m_shootPivotMotor = new TalonFX(Constants.kShootPivotId, "rio");
        

        m_absoluteEncoder = new DutyCycleEncoder(2);
        // m_absoluteEncoder.setDistancePerRotation(positionDeg);

        

        m_shootBeamBreaker = new DigitalInput(1);
        // m_note_shot_timings = new Notifier(this::report_change);

        // m_faster_tilt_pid = new Notifier(this::pidup);
        

        m_Timer = new Timer();
        m_Tiltcontroller.setTolerance(0.0001);
        
        m_Tiltcontroller.reset(getShootPivotAngle());
        m_Tiltcontroller.setGoal(0.483);

        // m_faster_tilt_pid.startPeriodic(0.01);
        
        
    }
    // private void report_change(){
    //     if( m_lastbeam != m_shootBeamBreaker.get()){
    //         System.out.println(Timer.getFPGATimestamp());
    //         m_lastbeam = ! m_lastbeam;
    //     }
    // }

    // public void start_detector(){
    //     m_note_shot_timings.startPeriodic(0);//run as fast as posible may be a mistatke 
    // }
    // public void stop_detector(){
    //     m_note_shot_timings.stop();
    // }

    //debugging --> encoder output, control the motor, PID values

    /* States */
    
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
    /* Sled Pivot */


     //actual angle = (actual voltage – 0deg voltage) * degrees_per_volt -> y = mx + b
    

    /*Shoot Pivot */ // -> for amp

    public double getPivotAbsPosition(){
        return m_absoluteEncoder.getAbsolutePosition();
    }

    public double getShootPivotAngle(){
        return m_absoluteEncoder.get();

        //straight valure = 0.487
        //maxtiltforward = 0.744
        //maxtiltbackword = 0.251
    }

    
    //TODO: figure out what the ideal pivot angle is


    public double getTargetTilt(){
        return targetAngle;
    }

    public void setTargetTilt(double angle){ //Should test to find what position to shoot at
        targetAngle = angle;
        m_Tiltcontroller.setGoal(targetAngle);
    }

    public void tiltToTarget(){
        m_Tiltcontroller.setGoal(targetAngle);
        m_shootPivotMotor.setControl(m_request.withVelocity(1));
    }
    
    public boolean isShootAligned(){
        //If our shoot is aligned, this statement is true, otherwise return false
        isShootAligned = getShootPivotAngle() == targetAngle;

        return isShootAligned;
    }
    public void tilt_strait()
    {
        m_Tiltcontroller.setGoal(0.487);
    }

    public boolean get_beam(){
        return m_shootBeamBreaker.get();
    }

    /* Commands */

    //isInShooter

    //isAlignedWithFeed

    //TODO: Change to RunEnd cmd(?); Add PID
    

    public void shootNote(){
        //if aligned, will shoot
        m_Tiltcontroller.setGoal(0.471);
        
        m_leftShootMotor.set(0.8); //between -1 and 1
        m_rightShootMotor.set(-0.8);

    }

    public void stop(){
        m_leftShootMotor.set(0);
        m_rightShootMotor.set(0);
    }

    public void set_shoot_speed(double speed){
        m_leftPID.setReference(speed, CANSparkBase.ControlType.kVelocity);
        m_rightPID.setReference(speed, CANSparkBase.ControlType.kVelocity);

        leftTargetSpd = speed;
        rightTargetSpd = speed;
    }
  
    public Command tilt_Shooter(Supplier<Double> shootangle){
        return run(() -> {
            setTargetTilt(shootangle.get());
        });
    }   

    private void pidup(){

        m_shootPivotMotor.setControl(motorRequest.withOutput(m_Tiltcontroller.calculate(getShootPivotAngle())));
    }

    @Override
    public void periodic(){

        //really volatile 

        double shootPivotControllerOutput = m_Tiltcontroller.calculate(getShootPivotAngle());
        T_shootPivotControllerOutput.set(shootPivotControllerOutput);
        m_shootPivotMotor.setControl(motorRequest.withOutput(shootPivotControllerOutput)); 

        T_shootBreak.set(get_beam());
        T_shootPivot.set(getShootPivotAngle());

        Logger.recordOutput("Left Speed", m_leftShootMotor.getEncoder().getVelocity());
        Logger.recordOutput("Right Speed", m_rightShootMotor.getEncoder().getVelocity());
        Logger.recordOutput("Left Shooter Setting", m_leftShootMotor.get());
        Logger.recordOutput("Right Shooter Setting", m_rightShootMotor.get());
        Logger.recordOutput("Pivot Angle", getShootPivotAngle());
        Logger.recordOutput("Pivot Error", m_Tiltcontroller.getPositionError());
        Logger.recordOutput("Left Velocity Error", leftTargetSpd - m_leftShootMotor.getEncoder().getVelocity());
        Logger.recordOutput("Right Velocity Error",  rightTargetSpd - m_rightShootMotor.getEncoder().getVelocity());
      

        
   
    }
}
