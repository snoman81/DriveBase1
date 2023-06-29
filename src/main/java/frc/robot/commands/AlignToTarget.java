// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;



public class AlignToTarget extends CommandBase {
  private PIDController m_LimeLightPIDController;
    private final DriveSubsystem m_DriveSubsystem;
    private final Limelight m_Limelight;
  private double setpoint;




  /** Creates a new AlignToTarget. */
  public AlignToTarget(DriveSubsystem m_DriveSubsystem, Limelight m_Limelight, double m_setpoint) {


    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_Limelight = m_Limelight;


  this.setpoint = setpoint;

  addRequirements(m_DriveSubsystem);
  addRequirements(m_Limelight);




    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_LimeLightPIDController = new PIDController (SmartDashboard.getNumber("LimeLightKP", Constants.LimeLightPID.LimeLightKP), SmartDashboard.getNumber("LimeLightKI", Constants.LimeLightPID.LimeLightKI),SmartDashboard.getNumber("KD", 0));
    m_LimeLightPIDController.setTolerance(Constants.LimeLightPID.tolerance);
  m_Limelight.initializeLimeLight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedforward = Constants.LimeLightPID.LimelightFF;
    double speed = m_LimeLightPIDController.calculate(m_Limelight.x(), setpoint);
    speed = (speed > 0)? speed + feedforward : speed - feedforward;
    speed = (speed > 1)? 1.0 : speed;
    speed = (speed < -1)? -1 : speed;


    m_DriveSubsystem.ArcadeDrive(0, speed*.125);

    SmartDashboard.putNumber("LimeLight Speed", speed);
    SmartDashboard.putNumber("LimeLight x Value", m_Limelight.x());
    SmartDashboard.putBoolean("Has Target?", m_Limelight.hasTargets());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;}
}