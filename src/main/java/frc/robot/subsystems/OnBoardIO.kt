// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * This class represents the onboard IO of the Romi reference robot. This includes the push buttons
 * and LEDs.
 *
 *
 * DIO 0 - Button A (input only) DIO 1 - Button B (input) or Green LED (output) DIO 2 - Button C
 * (input) or Red LED (output) DIO 3 - Yellow LED (output only)
 */
class OnBoardIO(dio1: ChannelMode, dio2: ChannelMode) : SubsystemBase() {
    private val buttonA = DigitalInput(0)
    private val yellowLed = DigitalOutput(3)

    // DIO 1
    private var buttonB: DigitalInput? = null
    private var greenLed: DigitalOutput? = null

    // DIO 2
    private var buttonC: DigitalInput? = null
    private var redLed: DigitalOutput? = null
    private var nextMessageTime = 0.0

    enum class ChannelMode {
        INPUT, OUTPUT
    }

    /**
     * Constructor.
     *
     * @param dio1 Mode for DIO 1 (input = Button B, output = green LED)
     * @param dio2 Mode for DIO 2 (input = Button C, output = red LED)
     */
    init {
        if (dio1 == ChannelMode.INPUT) {
            buttonB = DigitalInput(1)
            greenLed = null
        } else {
            buttonB = null
            greenLed = DigitalOutput(1)
        }
        if (dio2 == ChannelMode.INPUT) {
            buttonC = DigitalInput(2)
            redLed = null
        } else {
            buttonC = null
            redLed = DigitalOutput(2)
        }
    }

    /** Gets if the A button is pressed.  */
    val buttonAPressed: Boolean
        get() = buttonA.get()

    /** Gets if the B button is pressed.  */
    val buttonBPressed: Boolean
        get() {
            if (buttonB != null) {
                return buttonB!!.get()
            }
            val currentTime = Timer.getFPGATimestamp()
            if (currentTime > nextMessageTime) {
                DriverStation.reportError("Button B was not configured", true)
                nextMessageTime = currentTime + MESSAGE_INTERVAL
            }
            return false
        }

    /** Gets if the C button is pressed.  */
    val buttonCPressed: Boolean
        get() {
            if (buttonC != null) {
                return buttonC!!.get()
            }
            val currentTime = Timer.getFPGATimestamp()
            if (currentTime > nextMessageTime) {
                DriverStation.reportError("Button C was not configured", true)
                nextMessageTime = currentTime + MESSAGE_INTERVAL
            }
            return false
        }

    /** Sets the green LED.  */
    fun setGreenLed(value: Boolean) {
        if (greenLed != null) {
            greenLed!!.set(value)
        } else {
            val currentTime = Timer.getFPGATimestamp()
            if (currentTime > nextMessageTime) {
                DriverStation.reportError("Green LED was not configured", true)
                nextMessageTime = currentTime + MESSAGE_INTERVAL
            }
        }
    }

    /** Sets the red LED.  */
    fun setRedLed(value: Boolean) {
        if (redLed != null) {
            redLed!!.set(value)
        } else {
            val currentTime = Timer.getFPGATimestamp()
            if (currentTime > nextMessageTime) {
                DriverStation.reportError("Red LED was not configured", true)
                nextMessageTime = currentTime + MESSAGE_INTERVAL
            }
        }
    }

    /** Sets the yellow LED.  */
    fun setYellowLed(value: Boolean) {
        yellowLed.set(value)
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    companion object {
        private const val MESSAGE_INTERVAL = 1.0
    }
}