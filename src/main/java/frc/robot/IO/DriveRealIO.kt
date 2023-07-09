package frc.robot.IO

import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.Spark
import frc.robot.Robot
import frc.robot.sensors.RomiGyro

class DriveRealIO: DriveIO {
    val leftMotor = Spark(0)
    val rightMotor = Spark(1)

    val leftEncoder = Encoder(4, 5)
    val rightEncoder = Encoder(6, 7)

    private val diffDrive = DifferentialDrive(leftMotor, rightMotor)

    val gyro = RomiGyro()

    init {
        rightMotor.inverted = true

        leftEncoder.distancePerPulse =
            Math.PI * WHEEL_DIAMETER_INCH / COUNTS_PER_REVOLUTION
        rightEncoder.distancePerPulse =
            Math.PI * WHEEL_DIAMETER_INCH / COUNTS_PER_REVOLUTION
    }

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        inputs.gyroAngle = gyro.angleZ
        inputs.leftEncoderDistanceInch = leftEncoder.distance
        inputs.rightEncoderDistanceInch = rightEncoder.distance
    }

    override fun arcadeDrive(xAxisSpeed: Double, zAxisRotate: Double) {
        diffDrive.arcadeDrive(xAxisSpeed, zAxisRotate)
    }

    override fun reset() {
        leftEncoder.reset()
        rightEncoder.reset()
    }
    override fun gyroReset() {
        gyro.reset()
    }

    companion object {
        private const val COUNTS_PER_REVOLUTION = 1440.0
        private const val WHEEL_DIAMETER_INCH = 2.75591 // 70 mm
        private const val WHEEL_CIRCUMFERENCE_METERS = 0.07 * Math.PI
    }

}