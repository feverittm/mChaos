// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.utils.GearRatioHelper;
import java.io.File;

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
  public static final class DriveConstants {
    public static final int kLeftMotor1CANID = 1;
    public static final int kLeftMotor2CANID = 3;
    public static final int kRightMotor1CANID = 2;
    public static final int kRightMotor2CANID = 4;
    public static final boolean INVERT_LEFT = true;
    public static final boolean INVERT_RIGHT = false;

    public static final int[] kLeftEncoderDIOs = { 4, 5 };
    public static final int[] kRightEncoderDIOs = { 2, 3 };
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
  }

  public static final class LauncherConstants {
    public static final File launcherMapCSV = new File(Filesystem.getDeployDirectory(), "launcherMap.csv");

    // Hood
    public static final int HOOD_MOTOR_CANID = 7;
    public static final int HOOD_MOTOR_PDP_CHANNEL = 0;
    public static final boolean HOOD_MOTOR_INVERTED = false;
    public static final GearRatioHelper MOTOR_TO_HOOD = new GearRatioHelper(1, 30);
    public static final int[] HOOD_ENCODER_DIO_CHANNEL = { 7, 8 };
    public static final boolean HOOD_ENCODER_INVERTED = false;
    public static final int HOOD_LIMIT_SWITCH_DIO = 6;
    public static final boolean HOOD_LIMIT_SWITCH_TRUE_WHEN_ACTIVATED = false;
    public static final FeedbackDevice HOOD_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;
    public static final int HOOD_ENCODER_CPR = 2048;

    public static final double HOOD_KS = 1.5; // volts
    public static final double HOOD_KG_GAIN = 0.2; // volts * cos(hood angle)
    public static final double HOOD_KV = 1; // volts / (rad/s)
    public static final double HOOD_KA = 1; // volts / (rad/s^2)

    public static final double HOOD_KP = 12;
    public static final double HOOD_KI = 2;
    public static final double HOOD_KD = 8;

    public static final int FLYWHEEL_ENCODER_VELOCITY_SMOOTHING_SAMPLES = 10;
    public static final int HOOD_ENCODER_VELOCITY_SMOOTHING_SAMPLES = 10;

    public static final double HOOD_MAX_ANGLE = 1000;

    public static final double HOOD_MOMENT = 0.0086; // kg m^2
    public static final double HOOD_RADIUS_METERS = 0.5;
    public static final double HOOD_MASS_KG = 0.9;

    public static final double HOOD_ENCODER_ERROR_RADIANS_STDDEV = 0.001;

    // Flywheel

    public static final int FORWARD_FLYWHEEL_MOTOR_CANID = 5;
    public static final int AFT_FLYWHEEL_MOTOR_CANID = 6;

    public static final int FORWARD_FLYWHEEL_MOTOR_PDP_CHANNEL = 0;
    public static final int AFT_FLYWHEEL_MOTOR_PDP_CHANNEL = 0;

    public static final boolean FORWARD_FLYWHEEL_MOTOR_INVERTED = false;
    public static final boolean AFT_FLYWHEEL_MOTOR_INVERTED = false;

    public static final double FLYWHEEL_KS = 1.5; // volts
    public static final double FLYWHEEL_KV = 0.05; // volts / (rad/s)
    public static final double FLYWHEEL_KA = 0.01; // volts / (rad/s^2)

    public static final double FLYWHEEL_KP = 0.5;
    public static final double FLYWHEEL_KI = 0.000005;
    public static final double FLYWHEEL_KD = 0;

    public static final int kShooterTargetRPS = 5000;

    public static final double FLYWHEEL_MOMENT = 0.04; // kg m^2

    public static final GearRatioHelper MOTOR_TO_FLYWHEEL = new GearRatioHelper(18, 22);
  }

  public static final class IndexerConstants {
    public static final int INDEXER_MOTOR_CANID = 5;

    public static final int LOWER_BREAKBEAM_DIO = 1;
    public static final int UPPER_BREAKBEAM_DIO = 0;

    public static final boolean LOWER_BREAMBEAM_BALL_PRESENT_WHEN_TRUE = true;
    public static final boolean UPPER_BREAKBEAM_BALL_PRESENT_WHEN_TRUE = true;

    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 40;
  }

  public static final class LEDDisplayConstants {
    public static final int LED_PWM = 0;
    public static final int DEFAULT_LED_ANIMATION = 0;
    public static final int LAUNCHER_PREPARING_LED_ANIMATION = 0;
    public static final int LAUNCHER_LAUNCHING_LED_ANIMATION = 0;
  }

  public static final class AutoConstants {
    public static final double kTimeoutSeconds = 3;
    public static final double kDriveDistanceMeters = 2;
    public static final double kDriveSpeed = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
