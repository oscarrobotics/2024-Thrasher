package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    final double kGearRatio = 4.71; //4.71:1 presumably
    final int kWheelDiameterInches = 3;
    final double kPhysicalMaxSpeedMetersPerSecond = 5.0;


    public static final class Mod0{
        public static final String moduleName = "";
        public static final int driveCANId = 1;
        public static final int steerCANId = 2;
        public static final Rotation2d angularOffset = Rotation2d.fromDegrees(180.0);
        public static final Translation2d positionalOffset = new Translation2d(Units.inchesToMeters(22.5/2),Units.inchesToMeters(-22.5/2));
        public static final boolean isReversed = false;
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleName, driveCANId, steerCANId, angularOffset, positionalOffset, isReversed);
    }
    
    public static final class Mod1{
        public static final String moduleName = "";
        public static final int driveCANId = 3;
        public static final int steerCANId = 4;
        public static final Rotation2d angularOffset = Rotation2d.fromDegrees(90);
        public static final Translation2d positionalOffset = new Translation2d(Units.inchesToMeters(22.5/2),Units.inchesToMeters(22.5/2));
        public static final boolean isReversed = false;
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleName, driveCANId, steerCANId, angularOffset, positionalOffset, isReversed);
    }
    
    public static final class Mod2{
        public static final String moduleName = "";
        public static final int driveCANId = 5;
        public static final int steerCANId = 6;
        public static final Rotation2d angularOffset = Rotation2d.fromDegrees(270.0);
        public static final Translation2d positionalOffset = new Translation2d(Units.inchesToMeters(-22.5/2),Units.inchesToMeters(-22.5/2));
        public static final boolean isReversed = false;
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleName, driveCANId, steerCANId, angularOffset, positionalOffset, isReversed);
    }
    
    public static final class Mod3{
        public static final String moduleName = "";
        public static final int driveCANId = 7;
        public static final int steerCANId = 8;
        public static final Rotation2d angularOffset = Rotation2d.fromDegrees(0.0);
        
        public static final Translation2d positionalOffset = new Translation2d(Units.inchesToMeters(-22.5/2),Units.inchesToMeters(22.5/2));
        public static final boolean isReversed = false;
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(moduleName, driveCANId, steerCANId, angularOffset, positionalOffset, isReversed);
    }
}
