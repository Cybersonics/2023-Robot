// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int LeftStick = 0;
    public static final int RightStick = 1;
    public static final int OpController = 2;
  }

  public static class AutoConstants {
    public static final double kPhysicalMaxSpeedMetersPerSecond = 6;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
    public static final double kPXController = 5;
    public static final double kPYController = 5;
    public static final double kPThetaController = 5;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final double kGoToPointLinearP = 0;
    public static final double kGoToPointLinearF = 0.5;
    public static final double kGoToPointAngularP = 0;
    public static final double kGoToPointAngularF = 0;

    public static final double maxTrajectoryOverrunSeconds = 3;
    public static final double kMaxDistanceMetersError = 0.1;
    public static final double kMaxAngleDegreesError = 5;

  }

  public static class DriveConstants {
    public static final int FrontLeftSteer = 11;
    public static final int FrontLeftDrive = 1;
    public static final int FrontLeftEncoderOffset = 0;
    public static final int FrontRightSteer = 14;
    public static final int FrontRightDrive = 4;
    public static final int FrontRightEncoderOffset = 0;
    public static final int BackLeftSteer = 12;
    public static final int BackLeftDrive = 2;
    public static final int BackLeftEncoderOffset = 0;
    public static final int BackRightSteer = 13;
    public static final int BackRightDrive = 3;
    public static final int BackRightEncoderOffset = 0;

    public static final class ModuleConstants {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1 / 6.923;
      public static final double kTurningMotorGearRatio = 1 / 1024;
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
      public static final double kPTurning = 0.5;
    }

    public static final class FrameConstants {
      // Distance between right and left wheels
      public static final double kTrackWidth = Units.inchesToMeters(22);
      // Distance between front and back wheels
      public static final double kWheelBase = Units.inchesToMeters(24);

      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

      public static final double kPhysicalMaxSpeedMetersPerSecond = 6;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    }
  }

  public static final class ArmConstants {
    public static final int shoulderMotorId = 23;
    public static final int extensionMotorId = 24;

    public static int shoulderMotorCurrentLimit = 40;

    public static double armShoulderPosition = 318.4;
    public static double armExtensionPosition = 1247;

    public static double shoulderEncoderBottom = 1;
    public static double shoulderEncoderMax = 350;

    public static int extensionMotorCurrentLimit = 10;

    public static double extensionEncoderIn = 1025;
    public static double extensionEncoderOut = 1715;

  }

  public static final class IntakeConstants {
    public static final int intakeMotorId = 25;
  }
}
