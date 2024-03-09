package frc.robot.Commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.SwerveSubsystem;

public class Dummy_Auto extends SequentialCommandGroup{
    
    private Dummy_Auto(SwerveSubsystem swerve, Shooter shoot){
        
        addCommands(
            
        );
    }
}
