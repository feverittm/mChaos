// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 *
 * Simple mechanism with a roller that indexes the balls before they go into the flywheel.
 *
 * The index roller should be positioned between the two breakbeam sensors.
 * 
 * The jist is to watch for balls coming into the indexer using the upper breakbeam sensor,
 * then during the firing procedure we will spin up the flywheel and then pulse the index 
 * motor to move a ball into the launcher using the lower breakbeam to tell us when to stop
 * the index motor.
 */

public class Indexer extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IndexerConstants.INDEXER_MOTOR_CANID);

  private final DigitalInput lowerBreakbeam = new DigitalInput(IndexerConstants.LOWER_BREAKBEAM_DIO);

  private final DigitalInput upperBreakbeam = new DigitalInput(IndexerConstants.UPPER_BREAKBEAM_DIO);

  private final Timer m_timer = new Timer();

  public Indexer() {
    m_timer.reset();
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  /*
   * Stop the indexer motor
   */
  public void stopIndexer() {
    intakeMotor.set(0.0);
  }

  public void indexBall() {
    // pulse the index motor to move a ball into the lower breakbeam
    if (ballAtLowerBreakbeam()) { return; }
    if (ballAtUpperBreakbeam()) {
      m_timer.restart();
      intakeMotor.set(0.5);
      if (ballAtLowerBreakbeam() || m_timer.get() > 2.0) {
        stopIndexer();
        return;
      }
    }
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
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish the breakbeam state to telemetry.
    builder.addBooleanProperty("upper_breakbeam", () -> upperBreakbeam.get(), null);
    builder.addBooleanProperty("lower_breakbeam", () -> lowerBreakbeam.get(), null);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Upper Breakbeam", ballAtUpperBreakbeam());
    SmartDashboard.putBoolean("Lower Breakbeam", ballAtLowerBreakbeam());
  }
}
