// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        //0: Front for side 1: Back for side
        public static final int[] kLeftMotorPorts = new int[] { 1, 0 };
        public static final int[] kRightMotorPorts = new int[] { 14, 15 };
        Translation2d FrontLloc = new Translation2d(0.381, 0.381);
        Translation2d FrontRloc = new Translation2d(0.381, -0.381);
        Translation2d BackLloc = new Translation2d(-0.381, 0.381);
        Translation2d BackRloc = new Translation2d(-0.381, -0.381);
        public MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
  FrontLloc, FrontRloc, BackLloc, BackRloc
); // unused for now, can be used for pathweaver and stuff
      }
}
