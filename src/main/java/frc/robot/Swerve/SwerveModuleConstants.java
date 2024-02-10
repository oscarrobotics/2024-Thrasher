package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class SwerveModuleConstants {
    public final String moduleName;
    public final int driveCANId;
    public final int steerCANId;
    public final Rotation2d angularOffset;
    public final Translation2d positionalOffset;
    public final boolean isReversed;

    public SwerveModuleConstants(String moduleName, int driveCANId, int steerCANId, Rotation2d angularOffset, Translation2d positionalOffset, boolean isReversed){
        this.moduleName = moduleName;
        this.driveCANId = driveCANId;
        this.steerCANId = steerCANId;
        this.angularOffset = angularOffset;
        this.positionalOffset = positionalOffset;
        this.isReversed = isReversed;
    }
}
