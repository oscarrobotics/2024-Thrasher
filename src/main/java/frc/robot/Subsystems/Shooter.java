package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    //2 NEO Vortex Flexes for Shooter
    //Absolute Encoder
    public CANSparkFlex m_shootMotor;
    public CANSparkFlex m_rearMotor;

    public AbsoluteEncoder m_absoluteEncoder;

    public Shooter(){
        m_shootMotor = new CANSparkFlex(0, MotorType.kBrushless);
        m_absoluteEncoder = m_shootMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    }

    @Override
    public void periodic(){
    }
}
