/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Drivebase extends SubsystemBase {
  
  WPI_TalonSRX right = new WPI_TalonSRX(1);
  WPI_TalonSRX rightFollow = new WPI_TalonSRX(2);
  WPI_TalonSRX left = new WPI_TalonSRX(3);
  WPI_TalonSRX leftFollow = new WPI_TalonSRX(4);
  // AHRS gyro = new AHRS();
  public WPI_TalonSRX[] motors = {right, rightFollow, left, leftFollow};
  DifferentialDrive lmao = new DifferentialDrive(left, right);
  /**
   * Creates a new ExampleSubsystem.
   */
  public Drivebase() {
    leftFollow.follow(left);
    rightFollow.follow(right);
    SmartDashboard.putNumber("speed", 0.6);
    SmartDashboard.putNumber("steer", 0.5);
  }
  public void straight(double speed){
    right.set(speed);
    left.set(speed);
  }
  public void drive(){
    // double speed = -RobotContainer.stick.getRawAxis(1) * SmartDashboard.getNumber("speed", 0.6);
    // double steer = RobotContainer.steering.getRawAxis(0) * SmartDashboard.getNumber("steer", 0.5) * speed;
    // right.set(speed + steer);
    // left.set(speed - steer);
    lmao.curvatureDrive(-RobotContainer.stick.getRawAxis(1) * SmartDashboard.getNumber("speed", 0.6), RobotContainer.steering.getRawAxis(0) * SmartDashboard.getNumber("steer", 0.5), RobotContainer.steering.getRawButton(5) || RobotContainer.steering.getRawButton(6));
    SmartDashboard.putNumber("stick", RobotContainer.stick.getRawAxis(1));
  }
  public void shoot(){
    left.set(0.2);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
