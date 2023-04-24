// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IndexerConstants.INDEXER_MOTOR_PORT);

  private final DigitalInput lowerBreakbeam = new DigitalInput(IndexerConstants.LOWER_BREAKBEAM_DIO);

  private final DigitalInput upperBreakbeam = new DigitalInput(IndexerConstants.UPPER_BREAKBEAM_DIO);

  public Indexer() {
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   *Returns whether a ball is sensed at the lower breakbeam of the indexer.
   * 
   * @return Ball Present
   */
  public boolean ballAtLowerBreakbeam() {
    return IndexerConstants.LOWER_BREAMBEAM_BALL_PRESENT_WHEN_TRUE
        ? lowerBreakbeam.get()
        : !lowerBreakbeam.get();
  }

  /**
   * Returns whether a ball is sensed at the upper breakbeam of the indexer.
   *
   * @return The corrected value of the upper breakbeam.
   */
  public boolean ballAtUpperBreakbeam() {
    return IndexerConstants.UPPER_BREAKBEAM_BALL_PRESENT_WHEN_TRUE
        ? upperBreakbeam.get()
        : !upperBreakbeam.get();
  }

  public boolean hasBall() {
    return (ballAtLowerBreakbeam() || ballAtUpperBreakbeam());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Upper Breakbeam", ballAtUpperBreakbeam());
    SmartDashboard.putBoolean("Lower Breakbeam", ballAtLowerBreakbeam());
  }
}
