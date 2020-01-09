/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivebase extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX right = new WPI_TalonSRX(1);
  WPI_TalonSRX rightFollow = new WPI_TalonSRX(2);
  WPI_TalonSRX left = new WPI_TalonSRX(3);
  WPI_TalonSRX leftFollow = new WPI_TalonSRX(4);
  WPI_TalonSRX middle = new WPI_TalonSRX(5);
  
  public Drivebase(){
    leftFollow.follow(left);
    rightFollow.follow(right);
    left.setInverted(InvertType.InvertMotorOutput);
    leftFollow.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void straight(double speed){
    right.set(speed);
    left.set(speed);
  }

  public void side(double speed){
    middle.set(speed);
  }
}
