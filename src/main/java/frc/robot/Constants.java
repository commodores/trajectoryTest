// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMasterPort = 3;
        public static final int kLeftSlave0Port = 4;

        public static final int kRightMasterPort = 2;
        public static final int kRightSlave0Port = 1;

        public static final int kPigeonPort = 7;

        public static final int kEncoderCPR = 2048; //https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html
        public static final double kWheelDiameterMeters = 0.1016; //4 inches
        public static final double kGearReduction = 7;
        public static final double kEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR) / kGearReduction; 
        public static final double kWheelDistancePerPulse = kEncoderDistancePerPulse/ kGearReduction; //DISTANCE PER PULSE OF WHEEL= (OUTER CIRCUMFERENCE OF WHEEL)/(ENCODER CPR*GEAR REDUCTION)

        public static final double ksVolts = 0.699;
        public static final double kvVoltSecondsPerMeter = 2.36;
        public static final double kaVoltSecondsSquaredPerMeter = 0.22;
        public static final double kPDriveVel = 2.24;   // 2.29

        public static final double kTrackwidthMeters = 0.59825;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
