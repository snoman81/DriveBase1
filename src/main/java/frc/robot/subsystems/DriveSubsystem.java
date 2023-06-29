// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  public final WPI_TalonFX frontleft;
  public final WPI_TalonFX rearleft;
  public final WPI_TalonFX frontright;
  public final WPI_TalonFX rearright;

  public final MotorControllerGroup LeftDrive;
  public final MotorControllerGroup RightDrive;

  public final DifferentialDrive m_Drive;





  public DriveSubsystem() {

frontleft = new WPI_TalonFX(Constants.OperatorConstants.frontleftdriveID);
rearleft = new WPI_TalonFX(Constants.OperatorConstants.rearleftdriveID);
frontright = new WPI_TalonFX(Constants.OperatorConstants.frontrightdriveID);
rearright = new WPI_TalonFX(Constants.OperatorConstants.rearrightdriveID);

LeftDrive = new MotorControllerGroup(frontleft, rearleft);
LeftDrive.setInverted(true);
RightDrive = new MotorControllerGroup(frontright, rearright);
RightDrive.setInverted(false);

m_Drive = new DifferentialDrive(LeftDrive, RightDrive);
  }



public void ArcadeDrive(double xspeed, double rotatespeed) {

m_Drive.arcadeDrive(xspeed, rotatespeed);

}

public double getleftencoder(){
  return frontleft.getSelectedSensorPosition();
}


public void TankDrive(double leftspeed, double rightspeed){
  m_Drive.tankDrive(leftspeed, rightspeed);
}


public double getRightEncoder(){

  return frontright.getSelectedSensorPosition();

}

public void ResetEncoders(){

  frontleft.setSelectedSensorPosition(0);
  rearleft.setSelectedSensorPosition(0);
  frontright.setSelectedSensorPosition(0);
  rearleft.setSelectedSensorPosition(0);
}

  @Override
  public void periodic() {


SmartDashboard.putNumber("Left Encoder", frontleft.getSelectedSensorPosition());
SmartDashboard.putNumber ("Right Encoder", frontright.getSelectedSensorPosition());

    // This method will be called once per scheduler run
  }
}
