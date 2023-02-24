// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLight extends SubsystemBase {
    /** Creates a new LEDLight. */
    I2C m_led = new I2C(I2C.Port.kOnboard, 0x04);

    public LEDLight() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setColor(int color){
        m_led.write(0x04, color);
    }
}
