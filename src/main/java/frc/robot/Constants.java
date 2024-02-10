package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double swerveDeadband = 0.1;
    /* Drivetrain Constants */
      public static final double kGearRatio = 4.71; //4.71:1 presumably
      public static final int kWheelDiameterInches = 3;
      public static final double kPhysicalMaxSpeedMetersPerSecond = 3.0;
      public static final double kMaxRotSpeedRadPerSecond = 5.0;

      /* Charaterization values */
        //kS = static forces; increase forces until mechanism works kA = 
        public static final double SkS = 0.000;
        public static final double SkA = 0.000;
        public static final double SkV = 0.000;

      /* Conversion Factors */
      public static final double angleConversionFactor = 360.0 / kGearRatio;
    
//     public static final class SwerveKinematics{

//         /*drivetrain constants (meters)
//          * kTrackWidth = the width of the drivetrain, measured by the center of the wheels
//          * kWheelBase = the length of the drivetrain, measured by the center of the wheels
//          * kWheelCircumference = the circumference of the wheels
//         */
//         public static final double kTrackWidth = Units.inchesToMeters(23.5);
//         public static final double kWheelBase = Units.inchesToMeters(23.5);
//         public static final double kWheelCircumference = kWheelDiameterInches * Math.PI;

//     //     public static final Translation2d[] kModuleTranslations = {
//     //         new Translation2d(kWheelBase / 2, kTrackWidth / 2),
//     //         new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
//     //         new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
//     //         new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
//     //     };

//     //   public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);
//     // }

}
