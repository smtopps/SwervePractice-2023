// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSubsystem extends SubsystemBase {

  private final CANdle candle = new CANdle(Constants.CANDLE_ID, "canivore");
  //private final CANdle candle = new CANdle(Constants.CANDLE_ID);

  /** Creates a new CANdleSubsystem. */
  public CANdleSubsystem() {
    candle.configFactoryDefault();
    //candle.configBrightnessScalar(brightness);
    candle.configStatusLedState(false);
    candle.configV5Enabled(false);
    candle.configVBatOutput(VBatOutputMode.Off);
    setDefult();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLED(int r, int g, int b, int start, int end) {
    int count = end - start + 1;
    candle.setLEDs(r, g, b, 0, start, count);
  }

  public void setDefult() {
    if(DriverStation.getAlliance() == Alliance.Blue) {
      candle.setLEDs(0, 0, 255, 0, 0, 8); //51
    }else{
      candle.setLEDs(255, 0, 0, 0, 0, 8);
    }
  }
}
