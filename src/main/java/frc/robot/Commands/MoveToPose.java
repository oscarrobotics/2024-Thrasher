package frc.robot.Commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command; //CommandBase is deprecated to Command
import frc.robot.Swerve.SwerveSubsystem;
import frc.robot.PhotonCameraWrapper;

public class MoveToPose extends Command {
    
    public final SwerveSubsystem m_swerve;

    // public final PIDController translatePID, strafePID, rotatePID;

    public final double Vmax = 0; //max desired velocity for aligning
    public final Pose2d desiredPose; //our desired pose for the robot
    public final double travelTime; //time we want to elapse (depends on distance)

    public final Timer m_timer = new Timer();

    double xvelocity, yvelocity; //our x and y components for velocity (translation = x, strafe = y)
    Rotation2d thetaVelocity; //our rotational velocity

    public MoveToPose(Pose2d desiredPose, SwerveSubsystem swerve){
        this.m_swerve = swerve;

        // this.translatePID = new PIDController(0, 0, 0);
        // this.strafePID = new PIDController(0, 0, 0);
        // this.rotatePID = new PIDController(0, 0, 0);

        this.desiredPose = desiredPose;

        /*This is where we calculate everything for travelling from our current position to our target position. */

        //pose --> Sdelta x and Sdelta y
        //delta S = sqrt (delta x^2 + delta y^2)
        //delta t = delta S/vmax
        //Vx and Vy --> Vmax(deltay/s), Vmax(deltax/s)

        Transform2d deltaDistance = getDist(desiredPose);

        double xdist = deltaDistance.getX(); //x and y components of our pose distances
        double ydist = deltaDistance.getY();

        double totaldist = Math.sqrt(Math.pow(xdist, 2) + Math.pow(ydist,2)); //magnitude of S = sqrt (delta x^2 + delta y^2)
        travelTime = totaldist/Vmax; //delta t = magnitude of S/vmax
        
        xvelocity = Vmax * (ydist/totaldist);
        yvelocity = Vmax * (xdist/totaldist);
        thetaVelocity = getRotation(desiredPose, xdist, ydist, totaldist, travelTime);
    }

    /* TODO: ADD DOCUMENTATION */
    public Transform2d getDist(Pose2d desiredPose){

        //pose --> Sdelta x and Sdelta y
        //Vx and Vy --> Vmax(deltay/s), Vmax(deltax/s)

        Pose2d targetPose = desiredPose;
        Pose2d robotPose = m_swerve.m_poseEstimator.getEstimatedPosition();

        Transform2d deltaDistance = targetPose.minus(robotPose); //change in distance
        return deltaDistance;
    }

    /* TODO: ADD DOCUMENTATION */
    public Rotation2d getRotation(Pose2d desiredPose, double xdist, double ydist, double totaldist, double travelTime){
        Transform2d deltaDistance = getDist(desiredPose);
        Rotation2d theta = deltaDistance.getRotation(); //change in degrees

        Rotation2d rotVel = theta.times(1/travelTime); //delta theta * 1/delta t = delta w --> change in degrees over time

        return rotVel;
    }

    @Override
    public void initialize(){                   //Vizualizes camera on Shuffleboard
        CameraServer.startAutomaticCapture();
        m_swerve.drive(xvelocity, yvelocity, thetaVelocity.getRadians(), true);
    }

    @Override
    public boolean isFinished(){
        //if exceeds delta t: stop
        return m_timer.hasElapsed(travelTime);
    }

    @Override
    public void end(boolean interrupted){
        m_swerve.stop();
    }
}
