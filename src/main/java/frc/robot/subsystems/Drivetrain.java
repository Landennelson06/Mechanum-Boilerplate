// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftFMotor;
  private final CANSparkMax rightFMotor;
  private final CANSparkMax leftBMotor;
  private final CANSparkMax rightBMotor;
  private Double maxSpeed; 
  MecanumDrive drive;

  public Drivetrain() {
    leftFMotor = new CANSparkMax(DriveConstants.kLeftMotorPorts[0], MotorType.kBrushless);
    rightFMotor = new CANSparkMax(DriveConstants.kRightMotorPorts[0], MotorType.kBrushless);
    leftBMotor = new CANSparkMax(DriveConstants.kLeftMotorPorts[1], MotorType.kBrushless);
    rightBMotor = new CANSparkMax(DriveConstants.kRightMotorPorts[1], MotorType.kBrushless);
    
    //Might need to be changed for y'all
    rightFMotor.setInverted(true);
    rightBMotor.setInverted(true);

    //Set Idle mode to brake 
    leftFMotor.setIdleMode(IdleMode.kBrake);
    leftBMotor.setIdleMode(IdleMode.kBrake);
    rightFMotor.setIdleMode(IdleMode.kBrake);
    rightBMotor.setIdleMode(IdleMode.kBrake);

    drive = new MecanumDrive(leftFMotor, leftBMotor, rightFMotor, rightBMotor);
  }

  public void Drive(final XboxController controller){
    //Possible you might need to play with these negatives and possibly axis names.
    drive.driveCartesian(-controller.getLeftY(), controller.getLeftX(), -controller.getRightX());
  }
  public void DriveWOController(double y, double x, double z){
    drive.driveCartesian(-y, x, -z);
  }
  public void stop(){
    drive.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
