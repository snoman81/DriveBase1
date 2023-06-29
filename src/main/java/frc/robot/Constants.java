// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  
//Drive Motor IDs

public static final int frontrightdriveID = 1;
public static final int rearrightdriveID = 2;
public static final int frontleftdriveID = 3;
public static final int rearleftdriveID = 4;

//Max Speeds

public static final double Max_Drive_Speed = 1;/////////////////////////////////////////////////////////////////////////////////////////////////
public static final double Max_Rotate_Speed = 0.5;/////////////////////////////////////////////////////////////////////////////////////////////////


  
 
  }


public static final class DrivePID{

    public static final double DrivekPL = .0005;
    public static final double DrivekIL = .000338;
    public static final double DrivekDL = 0.0;
    public static final double DriveFFL = 0.0;
    public static final double toleranceL = 0;


    public static final double DrivekPR = .0005;
    public static final double DrivekIR = .000338;
    public static final double DrivekDR = 0.0;
    public static final double DriveFFR = 0.0;
    public static final double toleranceR = 0;
  }

public static final class LimeLightPID{
  public static final double LimeLightKP = .005;
  public static final double LimeLightKI = .0;
public static final double LimelightFF = 0;
  public static double tolerance;

}

public static final String limelightPID = null;




}