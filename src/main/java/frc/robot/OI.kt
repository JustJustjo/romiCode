package frc.robot

import edu.wpi.first.wpilibj.Joystick

object OI {
    val driverController = CustomController(0)
}
class CustomController(port: Int): Joystick(port) {
    val a: Boolean
        get () = getRawButton(1)
    val b: Boolean
        get() = getRawButton(2)
    val x: Boolean
        get() = getRawButton(3)
    val y: Boolean
        get() = getRawButton(4)
    val leftStickX: Double
        get() = getRawAxis(0)
    val leftStickY: Double
        get() = getRawAxis(1)
    val rightStickX: Double
        get() = getRawAxis(4)
    val rightStickY: Double
        get() = getRawAxis(5)
}