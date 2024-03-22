package frc.robot.Subsystems;

import frc.robot.Constants;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

    private TrapezoidProfile.Constraints m_Tiltconstraints = new TrapezoidProfile.Constraints(120,240);

    private ProfiledPIDController m_Tiltcontroller = 
        // new ProfiledPIDController(3, 0.003, 0.045, m_Tiltconstraints, 0.02);
        new ProfiledPIDController(3, 0.000, 0.0, m_Tiltconstraints, 0.02);

    

    private final double tilt_kFF = 0.0;
         

    private final VelocityVoltage m_request = new VelocityVoltage(0);

    // private DutyCycleOut motorRequest = new DutyCycleOut(0.0); 
    private VoltageOut motorRequest = new VoltageOut(0.0); 

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

        m_leftPID.setP(0.00005);
        m_leftPID.setI(0.0000003);
        m_leftPID.setIZone(100);
        m_leftPID.setD(0.0000001);
        m_leftPID.setFF(0.00017);

        m_rightPID.setP(0.00004);//0.000045
        m_rightPID.setI(0.000000035);//0.00000015
        m_leftPID.setIZone(100);
        m_rightPID.setD(0.0000001);
        m_rightPID.setFF(0.000165); //12V / 6000 RPM

        
        //sled pivot
        


        // m_sledPivotMotor = new TalonFX(1);
        m_shootPivotMotor = new TalonFX(Constants.kShootPivotId, "rio");
        m_shootPivotMotor.optimizeBusUtilization();
        

        m_absoluteEncoder = new DutyCycleEncoder(2);
        // m_absoluteEncoder.setDistancePerRotation(positionDeg);

        

        m_shootBeamBreaker = new DigitalInput(1);
        // m_note_shot_timings = new Notifier(this::report_change);

        // m_faster_tilt_pid = new Notifier(this::pidup);
        

        m_Timer = new Timer();
        m_Tiltcontroller.setTolerance(0.0001);
        
        m_Tiltcontroller.reset(getShootPivotAngle());
        m_Tiltcontroller.setGoal(0.483);

        // m_faster_tilt_pid.startPeriodic(0.02);
        
        
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

    // public void tiltToTarget(){
    //     m_Tiltcontroller.setGoal(targetAngle);
    //     m_shootPivotMotor.setControl(m_request.withVelocity(1));
    // }
    
    public boolean isShootAligned(){
        //If our shoot is aligned, this statement is true, otherwise return false
        isShootAligned =  Math.abs(getShootPivotAngle()  -m_Tiltcontroller.getGoal().position)<=0.007;

        return isShootAligned;
    }
    public void tilt_strait()
    {
        m_Tiltcontroller.setGoal(0.483);
    }
    public void tilt_amp()
    {
        m_Tiltcontroller.setGoal(0.683);
    }

    public boolean get_beam(){
        return m_shootBeamBreaker.get();
    }

    /* Commands */

    //isInShooter

    //isAlignedWithFeed

    //TODO: Change to RunEnd cmd(?); Add PID
    

    // public void shootNote(){
    //     //if aligned, will shoot
    //     // m_Tiltcontroller.setGoal(0.471);
        
        
    //     m_leftShootMotor.set(0.8); //between -1 and 1
    //     m_rightShootMotor.set(-0.8);

    // }
    public void shootNote(){
        //if aligned, will shoot
        // m_Tiltcontroller.setGoal(0.471);
        
        
        m_leftShootMotor.set(0.8); //between -1 and 1
        m_rightShootMotor.set(-0.8);

    }

    public void shootNote_speed(){
        set_wheel_speeds(5000);
    }

    public void shootNote_speed(double speed){
        set_wheel_speeds(speed);
    }

    public void stop(){
        m_leftShootMotor.set(0);
        m_rightShootMotor.set(0);
        leftTargetSpd = 0;
        rightTargetSpd = 0;
    }

    public void zero_speed(){
        set_wheel_speeds(0);
        leftTargetSpd = 0;
        rightTargetSpd = 0;
    }

    public void set_wheel_speeds(double leftspeed, double rightspeed){
        m_leftPID.setReference(leftspeed, CANSparkBase.ControlType.kVelocity);
        m_rightPID.setReference(-rightspeed, CANSparkBase.ControlType.kVelocity);

        leftTargetSpd = leftspeed;
        rightTargetSpd = -rightspeed;
    }
    public void set_wheel_speeds(double speed){
        m_leftPID.setReference(speed, CANSparkBase.ControlType.kVelocity);
        m_rightPID.setReference(-speed, CANSparkBase.ControlType.kVelocity);

        leftTargetSpd = speed;
        rightTargetSpd = -speed;
}

    public void loadnote(){
      set_wheel_speeds(950);
    }

    public void unload_amp(){
        set_wheel_speeds(-900);

    }
  
    public Command tilt_Shooter(Supplier<Double> shootangle){
        return run(() -> {
            setTargetTilt(shootangle.get());
        });
    }   

    private void pidup(){

       double shootPivotControllerOutput = m_Tiltcontroller.calculate(getShootPivotAngle())+(m_Tiltcontroller.getGoal().position*tilt_kFF)-(tilt_kFF*0.44);
        // T_shootPivotControllerOutput.set(shootPivotControllerOutput);
        m_shootPivotMotor.setControl(motorRequest.withOutput(12*shootPivotControllerOutput)); 
    }

    @Override
    public void periodic(){

        //really volatile 

        double shootPivotControllerOutput = m_Tiltcontroller.calculate(getShootPivotAngle())+(m_Tiltcontroller.getGoal().position*tilt_kFF)-(tilt_kFF*0.44);
        // T_shootPivotControllerOutput.set(shootPivotControllerOutput);
        m_shootPivotMotor.setControl(motorRequest.withOutput(12*shootPivotControllerOutput)); 

        // T_shootBreak.set(get_beam());
        // T_shootPivot.set(getShootPivotAngle());
        Logger.recordOutput("shoot_beam", get_beam());
        Logger.recordOutput("Left Speed", m_leftShootMotor.getEncoder().getVelocity());
        Logger.recordOutput("Right Speed", m_rightShootMotor.getEncoder().getVelocity());
        Logger.recordOutput("Left Shooter Setting", leftTargetSpd);
        Logger.recordOutput("Right Shooter Setting", rightTargetSpd);
        Logger.recordOutput("Pivot Angle", getShootPivotAngle());
        Logger.recordOutput("Pivot Error", m_Tiltcontroller.getPositionError());
        Logger.recordOutput("Pivot Target", m_Tiltcontroller.getGoal().position);
        Logger.recordOutput("Left Velocity Error", leftTargetSpd - m_leftShootMotor.getEncoder().getVelocity());
        Logger.recordOutput("Right Velocity Error",  rightTargetSpd - m_rightShootMotor.getEncoder().getVelocity());
      

        
   
    }
}
