package frc.robot.Swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule {

    public String moduleName;
    public int moduleNumber;
    public Rotation2d lastAngle;
    public Rotation2d angularOffset;
    public Translation2d positionalOffset;
    /* Motors/Encoders */

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;

    private RelativeEncoder m_driveEncoder;
    private AbsoluteEncoder m_integratedSteerEncoder;
    // private AbsoluteEncoder m_steerEncoder;

    private SparkPIDController m_drivePIDController;
    private SparkPIDController m_steerPIDController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SkS, Constants.SkV, Constants.SkA);

    /* Boolean conditions in case the positions are 
    reversed to how we want it when physically starting the bot */
    private boolean isAbsEncoderReversed;
    private double absoluteEncoderOffsetRad;


    //Constructor
    //pass in constants
    public SwerveModule(String moduleName, int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleName = moduleName;
        this.moduleNumber = moduleNumber;
        angularOffset = moduleConstants.angularOffset;
        positionalOffset = moduleConstants.positionalOffset;

        m_driveMotor = new CANSparkMax(moduleConstants.driveCANId, CANSparkLowLevel.MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(moduleConstants.steerCANId, CANSparkLowLevel.MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_steerMotor.restoreFactoryDefaults();

        // m_integratedSteerEncoder = m_steerMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        m_integratedSteerEncoder = m_steerMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_driveEncoder = m_driveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        // m_steerEncoder = m_steerMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        m_drivePIDController = m_driveMotor.getPIDController();
        m_steerPIDController = m_steerMotor.getPIDController();
        m_drivePIDController.setFeedbackDevice(m_driveEncoder);
        m_steerPIDController.setFeedbackDevice(m_integratedSteerEncoder);

        m_driveEncoder.setPositionConversionFactor(0.05077956125529683);
        m_driveEncoder.setVelocityConversionFactor(0.0008463260209216138);

        m_integratedSteerEncoder.setPositionConversionFactor(6.283185307179586);
        m_integratedSteerEncoder.setVelocityConversionFactor(0.10471975511965977);

        m_integratedSteerEncoder.setInverted(true);

        m_steerPIDController.setPositionPIDWrappingEnabled(true);
        m_steerPIDController.setPositionPIDWrappingMinInput(0);
        m_steerPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        m_drivePIDController.setP(0.000);
        m_drivePIDController.setI(0.000);
        m_drivePIDController.setD(0.000);
        m_drivePIDController.setFF(0.25);

        m_steerPIDController.setP(1);
        m_steerPIDController.setI(0.000);
        m_steerPIDController.setD(0.000);
        m_steerPIDController.setFF(0.000);

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_steerMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setSmartCurrentLimit(50);
        m_steerMotor.setSmartCurrentLimit(20);

        m_driveMotor.burnFlash();
        m_steerMotor.burnFlash();

        resetEncoders();
    }
    //Controller --> Chassispeeds --> Swerve Module State [x4]

    //get abs angle


    //optimize turning
    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double targetAngle = orientTo2Pi(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if(Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }


    //orient angle
    public double orientTo2Pi(double curAngle, double newAngle){
        double lowerBound;
        double upperBound;
        double lowerOffset = curAngle % 360;

        //Whether the angle is above a full rotation
        if(lowerOffset >= 0){
            lowerBound = curAngle - lowerOffset;
            upperBound = curAngle + (360 - lowerOffset);
        } else {
            upperBound = curAngle - lowerOffset;
            lowerBound = curAngle + (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
            newAngle += 360;
        }
        while(newAngle > upperBound){
            newAngle -= 360;
        }
        if(newAngle - curAngle > 180){
            newAngle += 360;
        }
        return newAngle;
    }

    //set velocity and turning angle

    /* Open loop: no position feedback while moving (useful for teleop) 
     * Closed loop: position feedback (useful for auto)
    */
    private void setVelocity(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond;
            m_driveMotor.set(percentOutput);
        } else {
            m_drivePIDController.setReference(
                desiredState.speedMetersPerSecond,
                CANSparkBase.ControlType.kVelocity,
                0,
                feedforward.calculate(desiredState.speedMetersPerSecond));
            }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = 
            (Math.abs(desiredState.speedMetersPerSecond) 
                <= (Constants.kPhysicalMaxSpeedMetersPerSecond * 0.01)) 
                ? lastAngle : desiredState.angle;
        m_steerPIDController.setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);
    }

    //config motor offset (in the case the motor and encoder are not aligned)


    //Zero out motors
    public void resetEncoders(){
        m_driveEncoder.setPosition(0);
    }

    //Subject to change if we find that accounting for the Absolute Encoder's angle
    //at all times is preferable to the internal encoder's
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getAbsoluteAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(), getAbsoluteAngle());
    }
   
    public double getAngle(){
        return m_integratedSteerEncoder.getPosition();
    }
  

    public Rotation2d getAbsoluteAngle(){ 
        return Rotation2d.fromRadians(m_integratedSteerEncoder.getPosition());
    }

    public double getDriveVelocity(){
        return m_driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return m_integratedSteerEncoder.getVelocity();
    }

    public double getMotorAppliedOutput(){
        return m_driveMotor.getAppliedOutput();
    }

    //Get Distance Command

    // /* (Given this is an analog input), you can divide the encoder's voltage reading by 
    //  * the amount supplied to obtain its percent of a full rotation.
    //  * 
    //  * Simply multiply by 2pi and subtract the offset to obtain the current angle
    //  * in radians.
    //  */
    // public double getAbsoluteEncoderRad(){
    //     double theta = m_absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    //     theta *= (2.0 * Math.PI);
    //     theta -= absoluteEncoderOffsetRad;
    //     return theta * (isAbsEncoderReversed ? -1.0 : 1.0);

    // }

       //Set desired state (based on utilizing the SparkMAX PID Controllers)
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace){
        desiredState.angle = desiredState.angle.plus(angularOffset);
        desiredState = optimize(desiredState, getState().angle);
        // m_driveMotor.set(desiredState.speedMetersPerSecond / kPhysicalMaxSpeedMetersPerSecond);

        // System.out.println(desiredState.angle.getRadians());
        
        // m_drivePIDController.setReference(desiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        // m_drivePIDController.setReference(desiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        m_drivePIDController.setReference(desiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
        m_steerPIDController.setReference(desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    }

    
     //m/s to rpm
     //Velocity(m/s) / wheel circumference (meters/revolution) * gear ratio (unitless) * 60 (seconds) --> dimensional analysis
    // private double VelocitytoRPM(double velocityMetersPerSecond){

    //     return (velocityMetersPerSecond * 60 * kGearRatio) / ((Units.inchesToMeters(kWheelDiameterInches) * Math.PI));
 
    // }

    public void stop(){
        m_driveMotor.set(0);
        m_steerMotor.set(0);
    }

    public void periodic(){
        SmartDashboard.putNumber("Velocity", getDriveVelocity());
    }
}
