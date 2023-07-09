package frc.robot.IO

import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim

class DriveSimIO : DriveIO {
    private val sim = DifferentialDrivetrainSim.createKitbotSim(DifferentialDrivetrainSim.KitbotMotor.kSingleNEOPerSide, DifferentialDrivetrainSim.KitbotGearing.k10p71, DifferentialDrivetrainSim.KitbotWheelSize.kSixInch, null )

    override fun updateInputs(inputs: DriveIO.DriveIOInputs) {
        sim.update(0.02)
        inputs.gyroAngle = sim.heading.degrees
        inputs.rightEncoderDistanceInch = sim.rightPositionMeters.metersToInches()
        inputs.leftEncoderDistanceInch = sim.leftPositionMeters.metersToInches()
    }

    override fun arcadeDrive(xAxisSpeed: Double, zAxisRotate: Double) {

        val xSpeed = MathUtil.applyDeadband(xAxisSpeed, 0.02)
        val zRotation = MathUtil.applyDeadband(zAxisRotate, 0.02)

        val speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true)

        sim.setInputs(MathUtil.clamp(speeds.left * 1.0, -12.0, 12.0), MathUtil.clamp(speeds.right * 1.0, -12.0, 12.0))

    }

    fun Double.metersToInches(): Double {
        return this*39.37
    }

}