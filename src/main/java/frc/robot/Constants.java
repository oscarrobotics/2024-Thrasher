package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.util.AllianceFlipUtil;

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

 public class VisionK {
  public static final Transform3d kFrontTagCamLocation = new Transform3d(
      0.5, 0.5, 0.25, new Rotation3d());

  public static final Transform3d kRearTagCamLocation = new Transform3d(
      -0.5, 0.5, 0.25, new Rotation3d(0, 0, Units.degreesToRadians(180)));

  public static final Transform3d kRightTagCamLocation = new Transform3d(
      0.5, 0.5, 0.25, new Rotation3d());

  public static final Transform3d kLeftTagCamLocation = new Transform3d(
      -0.5, 0.5, 0.25, new Rotation3d(0, 0, Units.degreesToRadians(180)));


  public static final Transform3d kFrontObjCamLocation = new Transform3d(
      0.5, 0.5, 0.25, new Rotation3d());

  public static final Transform3d kRearObjCamLocation = new Transform3d(
      -0.5, 0.5, 0.25, new Rotation3d(0, 0, Units.degreesToRadians(180)));

  public static final Transform3d kRightObjCamLocation = new Transform3d(
      0.5, 0.5, 0.25, new Rotation3d());

  public static final Transform3d kLeftObjCamLocation = new Transform3d(
      -0.5, 0.5, 0.25, new Rotation3d(0, 0, Units.degreesToRadians(180)));
}

public class FieldK {
        public static final Measure<Distance> kFieldLength = Meters.of(16.54);
        public static final Measure<Distance> kFieldWidth = Meters.of(8.21);

        public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // taken from 6328. All in blue alliance origin.
        /* speaker constants */
        public static final class SpeakerK {
            private static final Measure<Distance> kTopX = Inches.of(18.055);
            private static final Measure<Distance> kTopZ = Inches.of(83.091);
            public static final Translation3d kTopRight = new Translation3d(
                kTopX, Inches.of(238.815), kTopZ);
            public static final Translation3d kTopLeft = new Translation3d(
                kTopX, Inches.of(197.765), kTopZ);

            private static final Measure<Distance> kBotX = Inches.of(0);
            private static final Measure<Distance> kBotZ = Inches.of(78.324);
            // private static final Translation3d kBotRight = new Translation3d(
            // kBotX, Inches.of(238.815), kBotZ);
            public static final Translation3d kBotLeft = new Translation3d(
                kBotX, Inches.of(197.765), kBotZ);

            public static final Translation3d kBlueCenterOpening = kBotLeft.interpolate(kTopRight, 0.5);
            public static final Pose3d kBlueCenterOpeningPose3d = new Pose3d(
                kBlueCenterOpening, new Rotation3d());
            public static final Translation3d kRedCenterOpening = AllianceFlipUtil.flip(kBlueCenterOpening);
            public static final Pose3d kRedCenterOpeningPose3d = new Pose3d(
                kRedCenterOpening, new Rotation3d());

            public static final Measure<Distance> kAimOffset = Inches.of(25);
        }

        /* amp constants */
        public static final Measure<Distance> kXToAmp = Inches.of(49.5);
        public static final Measure<Distance> kYToAmp = Inches.of(286.765);
        public static final Measure<Distance> kZToAmp = Inches.of(35);
        public static final Measure<Distance> kZToSpeaker = Inches.of(98.13);

        public static final Translation3d kBlueAmpPose = new Translation3d(
            kXToAmp, kYToAmp, kZToSpeaker);

        public static final Translation3d kRedAmpPose = new Translation3d(
            kFieldLength.minus(kXToAmp), kFieldWidth.minus(kYToAmp), kZToAmp);

        /* stage constants */
        public static final double kBlueStageClearanceDs = Units.inchesToMeters(188.5);
        public static final double kBlueStageClearanceRight = Units.inchesToMeters(88.3);
        public static final double kBlueStageClearanceCenter = Units.inchesToMeters(243.2);
        public static final double kBlueStageClearanceLeft = Units.inchesToMeters(234.9);

        public static final double kRedStageClearanceDs = Units.inchesToMeters(542.2);
        public static final double kRedStageClearanceRight = Units.inchesToMeters(88.3);
        public static final double kRedStageClearanceCenter = Units.inchesToMeters(407.9);
        public static final double kRedStageClearanceLeft = Units.inchesToMeters(234.9);
    }

    
    public class IntakeK {
        public static final int kFrontIntakeId = 10;
        public static final int kRearIntakeId = 11;
    }


    public class ShooterK {
        public static final class ShooterConfigs {
            private static final double kPRight = 0.05;
            private static final double kSRight = 0.27; // sysid 0.15837
            private static final double kVRight = 0.0625;
            private static final double kARight = 0.007685;

            private static final double kPLeft = 0.0475; // sysid 0.046968
            private static final double kSLeft = 0.257; // sysid 0.2057
            private static final double kVLeft = 0.065; // sysid 0.052935
            private static final double kALeft = 0.017803;



            static {
                // kLeftConfigs.Feedback = kLeftConfigs.Feedback
                //     .withSensorToMechanismRatio(kGearRatio);

                // kLeftConfigs.Slot0 = kLeftConfigs.Slot0
                //     .withKP(kPRight)
                //     .withKS(kSRight)
                //     .withKV(kVRight)
                //     .withKA(kARight);

                // kLeftConfigs.MotorOutput = kLeftConfigs.MotorOutput
                //     .withNeutralMode(NeutralModeValue.Coast);

                // kRightConfigs.Feedback = kRightConfigs.Feedback
                //     .withSensorToMechanismRatio(kGearRatio);

                // kRightConfigs.Slot0 = kRightConfigs.Slot0
                //     .withKP(kPLeft)
                //     .withKS(kSLeft)
                //     .withKV(kVLeft)
                //     .withKA(kALeft);

                // kRightConfigs.MotorOutput = kRightConfigs.MotorOutput
                //     .withNeutralMode(NeutralModeValue.Coast);
            }
        }

        public static final String kDbTabName = "Shooter";

        public static final int kRightId = 13;
        public static final int kLeftId = 14;

        public static final double kSpinAmt = 0.7;

        public static final double kGearRatio = 1;

        public static final class FlywheelSimK {
            public static final double kMoi = 0.056699046875; // kg m^2

            // basically just how much faster the wheels have to spin for 1 meter more of
            // distance idk how this will work
            public static final double kRpmFactor = 100; // TODO fix this i just chose a random value
        }
    }

    public static class AimK {
        public static final class AimConfigs {
            private static final double kP = 5; // idk
            private static final double kS = 0.89;
            private static final double kG = 0;
            private static final double kV = 22.57; // V * s / rot
            private static final double kA = 0.12; // V * s^2 / rot
            private static final double kAcceleration = 160;

           


        }

        public static final String kDbTabName = "Aim";

        public static final int kAimId = 20;
        public static final int kHomeSwitch = 1;

        // 100:1 falconsport, 36:36 belt drive, 100:1 total
       
        public static final double kGearRatio = (100 * (36.0 / 36));

        public static final Measure<Distance> kLength = Inches.of(19.75);
        // asin((22 - kHeightTilShooter) / kLength)
        public static final Measure<Angle> kStageClearance = Degrees.of(47.097);
        public static final Measure<Angle> kMinAngle = Degrees.of(22.5);
        public static final Measure<Angle> kInitAngle = Degrees.of(90);
        public static final Measure<Angle> kMaxAngle = Degrees.of(150);
    }

    public static class SledK {
        public static final class SledConfigs {
            private static final double kP = 5; // idk
            private static final double kS = 0.89;
            private static final double kG = 0;
            private static final double kV = 22.57; // V * s / rot
            private static final double kA = 0.12; // V * s^2 / rot
            private static final double kAcceleration = 160;

           


        }

        public static final String kDbTabName = "Sled";

        public static final int kAimId = 99;
        // public static final int kHomeSwitch = 1;

        // 100:1 falconsport, 15:42 belt drive, 280 total
       
        public static final double kGearRatio = (100 * (15.0 / 42));

        public static final Measure<Distance> kLength = Inches.of(19.75);
        // asin((22 - kHeightTilShooter) / kLength)
        public static final Measure<Angle> kStageClearance = Degrees.of(47.097);
        public static final Measure<Angle> kMinAngle = Degrees.of(22.5);
        public static final Measure<Angle> kInitAngle = Degrees.of(90);
        public static final Measure<Angle> kMaxAngle = Degrees.of(150);
    }




    // public class ClimberK {
    // public static final int kLeftId = 20;
    // public static final int kRightId = 21;
    // public static final int kLimitSwitchId = 2;

    // public static final Measure<Distance> kMetersPerRotation = Meters.of(0.3);
    // public static final Measure<Distance> kMaxHeight = Inches.of(56);

    // public static final double kP = 3.25;
    // }

    public class RobotK {
        public static final Measure<Distance> kHeightTilShooter = Inches.of(7.533);
        public static final boolean kTestMode = false;
        public static final double kSimInterval = 0.020;
    }

}
