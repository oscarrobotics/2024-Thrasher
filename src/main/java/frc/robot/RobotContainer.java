package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Swerve.SwerveSubsystem;
import frc.robot.Commands.Intake_note;
import frc.robot.Commands.Outtake_note;
import frc.robot.Commands.Shoot_note;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Sled;

public class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final ControllerButtons m_operator = new ControllerButtons(1);
    
    public final SwerveSubsystem m_swerve = new SwerveSubsystem();

    public final Shooter m_shooter = new Shooter();

    public final Intake m_intake = new Intake();
     
    public final Sled m_sled = new Sled();

    public final Intake_note intake = new Intake_note(m_intake, m_sled);
    public final Outtake_note outtake = new Outtake_note(m_intake, m_sled);
    public final Shoot_note shoot = new Shoot_note(m_shooter, m_sled);

    // public Command shoot_note_properly = new runOnce(() -> m_sled.runSled()).waitUnti

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

    m_operator.arcadeWhiteLeft().onTrue(intake);
    m_operator.arcadeBlackLeft().onTrue(outtake);
    m_driverController.povUp().onTrue(m_sled.tilt_up());
    m_driverController.povDown().onTrue(m_sled.tilt_down());
    m_operator.arcadeWhiteRight().onTrue(shoot);
    // m_operator.a().onTrue() -> something to do with shoot or intake
      
    // Supplier<Double> leftslider = () -> m_operator.getRawAxis(0); 
    // Supplier<Double> rightslider = () -> m_operator.getRawAxis(1);
    
    m_sled.setDefaultCommand(
    
      m_sled.debug_runner(
        () -> m_operator.getLeftSlider()
      )

    );  

    m_shooter.setDefaultCommand(

      m_shooter.tilt_Shooter(() -> m_operator.getRightSlider()
      )

    );
  }


  
  public void teleopInit(){
    // m_swerve.resetOdometry();
    m_sled.resetSledPivot();
  } 

}
