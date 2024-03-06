package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

public class Intake extends SubsystemBase{
    //2 Falcons --> Intake
    public Timer m_timer;
    public TalonFX m_frontIntakeMotor, m_rearIntakeMotor;

    
    private boolean isStowed;
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    
    double frontWheelTargetSpeed;

    public Intake(){
        
        m_frontIntakeMotor = new TalonFX(Constants.kFrontIntakeId);
        m_rearIntakeMotor = new TalonFX(Constants.kRearIntakeId);

        TalonFXConfiguration config = new TalonFXConfiguration();  

        
        
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        config.Voltage.PeakForwardVoltage = 8;
        config.Voltage.PeakReverseVoltage = -8; 

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_frontIntakeMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
          }
          if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
          }
        
        
        for (int i = 0; i < 5; ++i) {
            status = m_rearIntakeMotor.getConfigurator().apply(config);
            if (status.isOK()) break;
          }
          if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
          }
    
        m_rearIntakeMotor.setInverted(true);




    }

       /* SLED */
    

    // ????????? Genuinely the worst thing I've ever seen
    // public Command stowedInSled(Supplier<Boolean> isStowed){
    //     return runOnce(() -> {isStowed.equals(isIn
        // Sled());});
    // }

    public void intake(){
        m_frontIntakeMotor.setControl(m_request.withVelocity(40));
        m_rearIntakeMotor.setControl(m_request.withVelocity(40));
    }
    public void outtake(){
        m_frontIntakeMotor.setControl(m_request.withVelocity(-40));
        m_rearIntakeMotor.setControl(m_request.withVelocity(-40));
    }   
    public void stop(){
        m_frontIntakeMotor.setControl(m_request.withVelocity(0));
        m_rearIntakeMotor.setControl(m_request.withVelocity(0));
    }
    public void rollforward(){
        m_frontIntakeMotor.setControl(m_request.withVelocity(40));
        m_rearIntakeMotor.setControl(m_request.withVelocity(-40));
    
    }
    public void rollbackward(){
        m_frontIntakeMotor.setControl(m_request.withVelocity(-40));
        m_rearIntakeMotor.setControl(m_request.withVelocity(40));
    }
    
    // final Supplier<Measure<Velocity<Angle>>> speed = () -> Rotations.per(Minute).of(30);
    // public Command intake(){
        
    //     return runEnd(() -> {
           
    //         m_frontIntakeMotor.setControl(m_request.withVelocity(40));
    //         m_rearIntakeMotor.setControl(m_request.withVelocity(40));
    //         // System.out.println("set motors");
    //         // m_sledMotor.setControl(m_request.withVelocity(0.5 * velocity.in(Rotations.per(Second))));
    //     }, () -> {
    //         m_frontIntakeMotor.setControl(m_request.withVelocity(0));
    //         m_rearIntakeMotor.setControl(m_request.withVelocity(0));
    //         // m_sledMotor.setControl(m_request.withVelocity(0));
    //     }).until(weAreStowed).withTimeout(2);
    //     // return run(()->{System.out.println("intake");}).withTimeout(0.1);
    // }

    // public Command reject(){
    //     return runEnd(() -> {
    //         m_frontIntakeMotor.setControl(m_request.withVelocity(-40));
    //         m_rearIntakeMotor.setControl(m_request.withVelocity(-40));
    //         // System.out.println("set motors");
    //         // m_sledMotor.setControl(m_request.withVelocity(0.5 * velocity.in(Rotations.per(Second))));
    //     }, () -> {
    //         m_frontIntakeMotor.setControl(m_request.withVelocity(0));
    //         m_rearIntakeMotor.setControl(m_request.withVelocity(0));
    //         // m_sledMotor.setControl(m_request.withVelocity(0));
    //     }).withTimeout(3);
    // }

    
    //Intake piece cmd
    //Reject piece cmd
}
