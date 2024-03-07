package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
import frc.robot.Constants.ShooterK;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;
import java.util.function.Supplier;

import javax.swing.text.Position;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    //2 NEO Vortex Flexes for Shooter
    //Absolute Encoder
    private Timer m_Timer;
 

    private DigitalInput m_shootBeamBreaker;

    private CANSparkFlex m_leftShootMotor, m_rightShootMotor;
    private SparkPIDController m_leftPID, m_rightPID;
    private TalonFX  m_shootPivotMotor;

    private DutyCycleEncoder m_absoluteEncoder;

    private TrapezoidProfile.Constraints m_Tiltconstraints = new TrapezoidProfile.Constraints(1,1);

    private ProfiledPIDController m_Tiltcontroller = 
        new ProfiledPIDController(0, 0, 0, m_Tiltconstraints, 0.02); 

    boolean isShootAligned;
    double targetAngle;

    // BooleanPublisher T_sledBreak;
    // BooleanPublisher T_inSled;
    // BooleanPublisher T_shootBreak;


    

    public Shooter(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable NT = inst.getTable("Shooter");
        
        
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

        m_Timer = new Timer();
        
    }

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
        //maxtiltforward = 0.873
        //maxtiltbackword = 0.177
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
    }
    
    public boolean isShootAligned(){
        //If our shoot is aligned, this statement is true, otherwise return false
        isShootAligned = getShootPivotAngle() == targetAngle;

        return isShootAligned;
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
        m_leftShootMotor.set(0.8); //between -1 and 1
        m_rightShootMotor.set(0.8);

    }

    public void stop(){
        m_leftShootMotor.set(0);
        m_rightShootMotor.set(0);
    }
    //shooter: shoot the note out

    @Override
    public void periodic(){

        //really volatile 

        // m_shootPivotMotor.setControl(motorRequest.withOutput(m_controller.calculate(m_absoluteEncoder.get())));
   
    }
}
