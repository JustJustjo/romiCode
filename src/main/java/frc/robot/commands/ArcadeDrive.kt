// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain
import java.util.function.Supplier

class ArcadeDrive(private val drivetrain: Drivetrain, private val xAxisSpeedSupplier: Supplier<Double>, private val zAxisRotateSupplier: Supplier<Double>) : CommandBase() {
    /**
     * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
     * lambdas. This command does not terminate.
     *
     * @param drivetrain          The drivetrain subsystem on which this command will run
     * @param xAxisSpeedSupplier  Lambda supplier of forward/backward speed
     * @param zAxisRotateSupplier Lambda supplier of rotational speed
     */
    init {
        addRequirements(drivetrain)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        drivetrain.romiArcadeDrive(xAxisSpeedSupplier.get(), zAxisRotateSupplier.get())
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}