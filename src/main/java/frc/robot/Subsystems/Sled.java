package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SynchronousInterrupt;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Sled extends SubsystemBase{
    //Analog Potentiometer
    //beam break sensor
    //talon pivot motor1
    //talon pivot motor2 (reversed)
    //!!make sure motors turn correct direction before going full power!!
    //talon belt motor 16/36
    private AnalogPotentiometer m_pivPotentiometer;
    private TalonFX m_leftSledPivotMotor, m_rightSledPivotMotor;


    private TrapezoidProfile.Constraints m_Sledconstraints = new TrapezoidProfile.Constraints(240, 80);

    private ProfiledPIDController m_Sledcontroller = 
        new ProfiledPIDController(0.08, 0.003, 0.002, m_Sledconstraints, 0.02); 

    private LinearFilter m_potfilter = LinearFilter.singlePoleIIR(0.01, 0.02);
    
    
    private VoltageOut motorRequest = new VoltageOut(0.0); 

    private TalonFX m_sledMotor;

    public DigitalInput m_sledBeamBreaker;

    DoublePublisher T_targetAngle;
    DoublePublisher T_sledPivotControllerOutput;

    DoublePublisher T_sledPivot;
    BooleanPublisher T_sledBreak;
    BooleanPublisher T_inSled;

    BooleanPublisher T_sensorBB;
    

    private final VelocityVoltage m_request = new VelocityVoltage(0);

    private final EventLoop eventLoop = new EventLoop();

    //Interrupt loop
    public boolean interruptRequest = false;
    private double sledBeamBreakIrqLastRising = 0;
    private double sledBeamBreakIrqLastFalling = 0;
    private final SynchronousInterrupt beamBreakInterrupt = new SynchronousInterrupt(m_sledBeamBreaker);
    // private final Trigger sledBeamBreakTrig;

    public Sled(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable NT = inst.getTable("Sled");
        T_sledPivot = NT.getDoubleTopic("sledPivot").publish();
        T_sledBreak = NT.getBooleanTopic("SledBreak").publish();
       
        T_inSled = NT.getBooleanTopic("inSled").publish();

        T_sensorBB = NT.getBooleanTopic("Sled BB Interruption").publish();

        m_leftSledPivotMotor = new TalonFX(Constants.kLeftSledPivotId);
        m_rightSledPivotMotor = new TalonFX(Constants.kRightSledPivotId);

        m_pivPotentiometer = new AnalogPotentiometer(1, 300, -11.9);
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();  

        // pivotConfig.Slot0.kP = 0.1;
        // pivotConfig.Slot0.kI = 0.0;
        // pivotConfig.Slot0.kD = 0.0;
        // pivotConfig.Slot0.kV = 0.12;

        pivotConfig.Voltage.PeakForwardVoltage = 2;
        pivotConfig.Voltage.PeakReverseVoltage = -2; 

        StatusCode pivotStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            pivotStatus = m_leftSledPivotMotor.getConfigurator().apply(pivotConfig);
            if (pivotStatus.isOK()) break;
          }
          if(!pivotStatus.isOK()) {
            System.out.println("Could not apply configs, error code: " + pivotStatus.toString());
          }
        m_leftSledPivotMotor.setInverted(true);
        m_rightSledPivotMotor.setControl(new Follower(Constants.kLeftSledPivotId ,true));
        // m_leftSledPivotMotor.setControl(new Follower(m_rightSledPivotMotor.getDeviceID(), true));
        m_leftSledPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_rightSledPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        
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
        
        // m_rightSledPivotMotor.optimizeBusUtilization();
        // m_leftSledPivotMotor.optimizeBusUtilization();
        // m_sledMotor.optimizeBusUtilization();

        m_Sledcontroller.reset(getSledPivotAngle());
        m_Sledcontroller.setGoal(getSledPivotAngle());

        m_potfilter.reset(new double[]{getPivotVoltage()},new double[]{getPivotVoltage()} 
        );

        m_sledBeamBreaker = new DigitalInput(0);


        // sledBeamBreakTrig = new Trigger(eventLoop, () -> interruptRequest);
        beamBreakInterrupt.setInterruptEdges(true, true);

        T_targetAngle = NT.getDoubleTopic("targetAngle").publish();
        T_sledPivotControllerOutput = NT.getDoubleTopic("sledPivotControllerOutput").publish();
    
        
    }


    public boolean get_beam(){
        return m_sledBeamBreaker.get();
    }
    public boolean isInSled(){
        // isStowed = (!m_intakeBeamBreaker.get())?true:false;//// stop this 
        // return isStowed; // and this
        return !m_sledBeamBreaker.get();

    }

    public double getSledPivotAngle(){
        return (m_pivPotentiometer.get());
        //min = 0 with -11.9 ofset
        //max = 56 with -11.9 ofset
    }

    public void setTargetSledPivot(double angle){
        angle = Math.max(angle, 0);
        angle = Math.min(angle, 55);
        T_targetAngle.set(angle);   
        m_Sledcontroller.setGoal(angle);
    }
    private double intakePose = 40;
    public void goToIntakePose(){
        m_Sledcontroller.setGoal(intakePose);
    }   
   
    public void resetSledPivot(){
        m_Sledcontroller.reset(getSledPivotAngle());
        m_Sledcontroller.setGoal(getSledPivotAngle());    
    }

    public double getPivotVoltage(){
        return m_pivPotentiometer.get();
    }

    public void runSled(){
        m_sledMotor.setControl(m_request.withVelocity(-80));
    }

    public void unrunSled(){
        m_sledMotor.setControl(m_request.withVelocity(80));
    }

    public void stop(){
        m_sledMotor.setControl(m_request.withVelocity(0));
    }


     public Command tilt_down(){
        if (getSledPivotAngle()<=50) {
            return runOnce(()->{setTargetSledPivot(getSledPivotAngle()+5);}).withTimeout(1);
            
        }
        return runOnce(()->{setTargetSledPivot(55);}).withTimeout(1);
        
    }

    public Command tilt_up(){
        if (getSledPivotAngle()>=5) {
            return runOnce(()->{setTargetSledPivot(getSledPivotAngle()-5);}).withTimeout(1);
            
        }
        return runOnce(()->{setTargetSledPivot(0);}).withTimeout(1);
        
    }

    public Command tilt_to(double angle){
        return runOnce(()->{setTargetSledPivot(angle);}).withTimeout(1);
    }


    public Command 
    rotateSled(Supplier<Double> sledangle){
        return run(
        () -> {setTargetSledPivot(sledangle.get());}
        );
        
    }

    private void evalSledInterrupt(){
        double curRising = beamBreakInterrupt.getRisingTimestamp();
        double curFalling = beamBreakInterrupt.getFallingTimestamp();

        boolean newRising = curRising > sledBeamBreakIrqLastRising;
        if(newRising){
            sledBeamBreakIrqLastRising = curRising;
        }

        boolean newFalling = curFalling > sledBeamBreakIrqLastFalling;
        if(newFalling){
            sledBeamBreakIrqLastFalling = curFalling;
        }

        if( curFalling > curRising && newRising){
            interruptRequest = true;
        }else if(curRising > curFalling && newRising){
            interruptRequest = false;
        }

        T_sensorBB.set(interruptRequest);
    }

    
    

    @Override
    public void periodic(){

        //really volatile 

        // m_shootPivotMotor.setControl(motorRequest.withOutput(m_controller.calculate(m_absoluteEncoder.get())));
        
        double sledPivotControllerOutput = m_Sledcontroller.calculate(m_potfilter.calculate(getSledPivotAngle()));
        // T_sledPivotControllerOutput.set(sledPivotControllerOutput);
        m_leftSledPivotMotor.setControl(motorRequest.withOutput(12*sledPivotControllerOutput)); 
        // SmartDashboard.putNumber("Piv outpt", sledPivotControllerOutput);
        SmartDashboard.putNumber("Angle", m_potfilter.lastValue()); 

        // T_sledBreak.set(get_beam());
        // T_sledPivot.set(m_potfilter.lastValue());
        
        // T_inSled.set(isInSled());  
        evalSledInterrupt();
        eventLoop.poll();

        Logger.recordOutput("sledBreak", get_beam());
        Logger.recordOutput("in_sled", isInSled());
        Logger.recordOutput("SledTarget",m_Sledcontroller.getGoal().position);
        Logger.recordOutput("SledPosition", m_potfilter.lastValue());
        Logger.recordOutput("SledError", m_Sledcontroller.getPositionError());
        Logger.recordOutput("Sled BB Interrupt", interruptRequest);
        // Logger.recordOutput("sled sped", n);
    }
    

}
