package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


    private TrapezoidProfile.Constraints m_Sledconstraints = new TrapezoidProfile.Constraints(4,3);

    private ProfiledPIDController m_Sledcontroller = 
        new ProfiledPIDController(0.03, 0, 0, m_Sledconstraints, 0.02); 
    
    private DutyCycleOut motorRequest = new DutyCycleOut(0.0); 

    private TalonFX m_sledMotor;

    public DigitalInput m_sledBeamBreaker;

    DoublePublisher T_targetAngle;
    DoublePublisher T_sledPivotControllerOutput;

    DoublePublisher T_sledPivot;
    BooleanPublisher T_sledBreak;
    BooleanPublisher T_inSled;
    BooleanPublisher T_shootBreak;

    private final VelocityVoltage m_request = new VelocityVoltage(0);


    public Sled(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable NT = inst.getTable("Sled");
        T_sledPivot = NT.getDoubleTopic("sledPivot").publish();
        T_sledBreak = NT.getBooleanTopic("SledBreak").publish();
        T_shootBreak = NT.getBooleanTopic("shootBreak").publish();
        T_inSled = NT.getBooleanTopic("inSled").publish();


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
        m_leftSledPivotMotor.setControl(new Follower(m_rightSledPivotMotor.getDeviceID(), true));
        
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

        m_Sledcontroller.reset(getSledPivotAngle());

        m_sledBeamBreaker = new DigitalInput(0);

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

    public void runSled(double velocity){
        m_sledMotor.setControl(m_request.withVelocity(velocity));
    }

    public void stopSled(){
        m_sledMotor.setControl(m_request.withVelocity(0));
    }

    public Command feed(){
        BooleanSupplier weAreStowed = () -> isInSled();
        return runOnce(() -> runSled(-90)).until(weAreStowed).andThen(() -> stopSled());
    }

    public Command unfeed(){
        return runOnce(() -> runSled(90)).withTimeout(2).andThen(() -> stopSled());
    }
    

    @Override
    public void periodic(){

        //really volatile 

        // m_shootPivotMotor.setControl(motorRequest.withOutput(m_controller.calculate(m_absoluteEncoder.get())));
        
        double sledPivotControllerOutput = m_Sledcontroller.calculate(getSledPivotAngle());
        T_sledPivotControllerOutput.set(sledPivotControllerOutput);
        m_leftSledPivotMotor.setControl(motorRequest.withOutput(sledPivotControllerOutput)); 
        // SmartDashboard.putNumber("Piv outpt", sledPivotControllerOutput);
        SmartDashboard.putNumber("Angle", getSledPivotAngle()); 

        T_sledBreak.set(get_beam());
        T_sledPivot.set(getSledPivotAngle());
        
        T_inSled.set(isInSled());  
    }
    

}
