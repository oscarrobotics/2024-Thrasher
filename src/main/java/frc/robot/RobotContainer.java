package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Swerve.SwerveSubsystem;
import frc.robot.Commands.Intake_note;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Mechanism;
import frc.robot.Subsystems.Sled;

public class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    // private final CommandXboxController m_operator = new CommandXboxController(1);
    private final ControllerButtons m_operator = new ControllerButtons(1);
    
    public final SwerveSubsystem m_swerve = new SwerveSubsystem();
    // public final Intake m_intake = new Intake();
    // public final Sled m_sled = new Sled();

    public final Mechanism m_mechanism = new Mechanism();

    public RobotContainer(){
    m_swerve.setDefaultCommand(
      m_swerve.teleopDrive(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> true,
        () -> true
      )
    );
    m_driverController.a().onTrue(new InstantCommand(() -> m_swerve.resetOdometry(), m_swerve) );
    m_driverController.b().whileTrue(
      m_swerve.evasiveDrive(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> true,
        () -> true
      )
      
    );

    // m_operator.arcadeWhiteLeft().onTrue(m_mechanism.intake());
    m_operator.arcadeWhiteLeft().onTrue(m_mechanism.intake());
    m_operator.arcadeBlackLeft().onTrue(m_mechanism.outtake());
    m_driverController.povUp().onTrue(m_mechanism.tilt_up());
    m_driverController.povDown().onTrue(m_mechanism.tilt_down());
    // m_operator.arcadeWhiteRight().onTrue(m_mechanism.shoot());
    // m_operator.a().onTrue() -> something to do with shoot or intake
      
    // Supplier<Double> leftslider = () -> m_operator.getRawAxis(0); 
    // Supplier<Double> rightslider = () -> m_operator.getRawAxis(1);
    
    m_mechanism.setDefaultCommand(
    
    
      m_mechanism.debug_runner(
        () -> m_operator.getLeftSlider()
      )
    );  
  }


  
  public void teleopInit(){
    // m_swerve.resetOdometry();
    m_mechanism.m_sled.resetSledPivot();
  } 

}
