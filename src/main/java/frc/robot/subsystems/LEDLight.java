// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLight extends SubsystemBase {
    public enum Color {
        OFF(0),
        PURPLE(1),
        YELLOW(2),
        ORANGE(3),
        PULSE(4);

        public int value;

        Color(int val) {
            this.value = val;
        }
    }

    /** Creates a new LEDLight. */
    I2C m_led = new I2C(I2C.Port.kOnboard, 0x04);

    public LEDLight() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setColor(Color color) {
        m_led.write(0x04, color.value);
    }
}
