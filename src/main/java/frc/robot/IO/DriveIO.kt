package frc.robot.IO

import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable

import org.littletonrobotics.junction.inputs.LoggableInputs





interface DriveIO {
    @AutoLog
    open class DriveIOInputs {
        @JvmField
        var gyroAngle = 0.0
        @JvmField
        var rightEncoderDistanceInch = 0.0
        @JvmField
        var leftEncoderDistanceInch = 0.0
    }
    class DriveIOInputsAutoLogged : DriveIOInputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("gyroAngle", gyroAngle)
            table.put("rightEncoderDistanceInch", rightEncoderDistanceInch)
            table.put("leftEncoderDistanceInch", leftEncoderDistanceInch)
        }

        override fun fromLog(table: LogTable) {
            gyroAngle = table.getDouble("gyroAngle", gyroAngle)
            rightEncoderDistanceInch = table.getDouble("rightEncoderDistanceInch", rightEncoderDistanceInch)
            leftEncoderDistanceInch = table.getDouble("leftEncoderDistanceInch", leftEncoderDistanceInch)
        }
    }

    fun updateInputs(inputs: DriveIOInputs) {}

    fun arcadeDrive(xAxisSpeed: Double, zAxisRotate: Double) {}

    fun reset() {}

    fun gyroReset() {}
}