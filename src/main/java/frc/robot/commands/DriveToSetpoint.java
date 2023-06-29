// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToSetpoint extends CommandBase {
private PIDController m_Left_Drive_Controller;
private PIDController m_Right_Drive_Controller;
  private final DriveSubsystem m_DriveSubsystem;
private double setpoint;

  /** Creates a new DriveToSetpoint. */
  public DriveToSetpoint(DriveSubsystem m_DriveSubsystem, double m_setpoint){

    SmartDashboard.setDefaultNumber("DriveKP", Constants.DrivePID.DrivekPL);
    SmartDashboard.setDefaultNumber("DriveKI", Constants.DrivePID.DrivekIL);

    SmartDashboard.setDefaultNumber("DriveKP", Constants.DrivePID.DrivekPR);
    SmartDashboard.setDefaultNumber("DriveKI", Constants.DrivePID.DrivekIR);
  
 // Use addRequirements() here to declare subsystem dependencies.
 this.m_DriveSubsystem = m_DriveSubsystem ;

this.setpoint = m_setpoint; 

addRequirements(m_DriveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    m_Left_Drive_Controller = new PIDController (SmartDashboard.getNumber("DriveKPL", Constants.DrivePID.DrivekPL), SmartDashboard.getNumber("DriveKIL", Constants.DrivePID.DrivekIL), SmartDashboard.getNumber("DriveKDL", Constants.DrivePID.DrivekDL));
    m_Left_Drive_Controller.setTolerance(Constants.DrivePID.toleranceL);

    m_Right_Drive_Controller = new PIDController (SmartDashboard.getNumber("DriveKPR", Constants.DrivePID.DrivekPR), SmartDashboard.getNumber("DriveKIR", Constants.DrivePID.DrivekIR), SmartDashboard.getNumber("DriveKDR", Constants.DrivePID.DrivekDR));
    m_Right_Drive_Controller.setTolerance(Constants.DrivePID.toleranceR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

double feedforward = Constants.DrivePID.DriveFFL;
double speed = m_Left_Drive_Controller.calculate(m_DriveSubsystem.getleftencoder(), setpoint);
speed = (speed > 0)? speed + feedforward : speed - feedforward;
speed = (speed > 1)? 1.0 : speed;
speed = (speed < -1)? -1 : speed;


double feedforward2 = Constants.DrivePID.DriveFFR;
double speed2 = m_Right_Drive_Controller.calculate(m_DriveSubsystem.getRightEncoder(), setpoint);
speed2 = (speed2 > 0)? speed2 + feedforward2 : speed2 - feedforward2;
speed2 = (speed2 > 1)? 1.0 : speed2;
speed2 = (speed2 < -1)? -1 : speed2;

m_DriveSubsystem.TankDrive(-speed*Constants.OperatorConstants.Max_Drive_Speed, speed2*Constants.OperatorConstants.Max_Drive_Speed);


SmartDashboard.putNumber("pidspeedL", speed);
SmartDashboard.putNumber("pidspeedR", speed2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
