package frc.robot.Commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command; //CommandBase is deprecated to Command
import frc.robot.Swerve.SwerveSubsystem;
import frc.robot.PhotonCameraWrapper;

public class AlignToAprilTag extends Command {
    
    public final SwerveSubsystem m_swerve;
    public final PhotonCameraWrapper m_vision;  //Not sure how to get camera info from PCW

    public final PIDController translatePID, strafePID, rotatePID;

    public AlignToAprilTag(){
        this.m_swerve = SwerveSubsystem.getInstance();
        this.m_vision = PhotonCameraWrapper.getInstance();

        this.translatePID = new PIDController(0, 0, 0);
        this.strafePID = new PIDController(0, 0, 0);
        this.rotatePID = new PIDController(0, 0, 0);
    }

    @Override
    public void initialize(){                   //Vizualizes camera on Shuffleboard
        CameraServer.startAutomaticCapture();
    }

    @Override
    public void execute(){
        double fwdVel = 0;
        double strafeVel = 0;
        double rotVel = 0;


    }
}
