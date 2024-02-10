package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Swerve.SwerveSubsystem;

public class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    
    public final SwerveSubsystem m_swerve = new SwerveSubsystem();

    public RobotContainer(){
    m_swerve.setDefaultCommand(
      m_swerve.teleopDrive(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> true,
        () -> true
      )
    );
    m_driverController.a().onTrue(new InstantCommand(() -> m_swerve.resetOdometry(), m_swerve) );
    m_driverController.b().whileTrue(
      m_swerve.evasiveDrive(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> true,
        () -> true
      )
      
    );
  }

}
