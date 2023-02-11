// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public static class DriveConstants {
    public static final int FrontLeftSteer = 11;
    public static final int FrontLeftDrive = 1;
    public static final int FrontLeftEncoderOffset = -1;
    public static final int FrontRightSteer = 14;
    public static final int FrontRightDrive = 4;
    public static final int FrontRightEncoderOffset = 12;
    public static final int BackLeftSteer = 12;
    public static final int BackLeftDrive = 2;
    public static final int BackLeftEncoderOffset = -5;
    public static final int BackRightSteer = 13;
    public static final int BackRightDrive = 3;
    public static final int BackRightEncoderOffset = 10;

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
}
