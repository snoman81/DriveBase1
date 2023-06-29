// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive extends CommandBase {
 
 
  /** Creates a new TeleopDrive. */

  public final DriveSubsystem m_DriveSubsystem;
  DoubleSupplier xspeed;
  DoubleSupplier rotatespeed;

  public TeleopDrive(DriveSubsystem m_DriveSubsystem, DoubleSupplier xspeed, DoubleSupplier rotationspeed) {

this.rotatespeed = rotationspeed;
this.xspeed = xspeed;
this.m_DriveSubsystem = m_DriveSubsystem;

addRequirements(m_DriveSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

m_DriveSubsystem.ArcadeDrive(xspeed.getAsDouble()*Constants.OperatorConstants.Max_Drive_Speed, rotatespeed.getAsDouble()*Constants.OperatorConstants.Max_Rotate_Speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

m_DriveSubsystem.ArcadeDrive( 0, 0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
