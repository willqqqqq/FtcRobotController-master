package org.firstinspires.ftc.teamcode.subsystemsCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystemsCommands.willDriveSubsystem;

import java.util.function.DoubleSupplier;

public class willDriveCommand extends CommandBase {

    private willDriveSubsystem driveSubsystem;
    private DoubleSupplier strafe, forward, turn;

    public willDriveCommand(willDriveSubsystem driveSubsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){

        this.driveSubsystem = driveSubsystem;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;

        addRequirements(driveSubsystem);

    }
    @Override
    public void execute(){

        driveSubsystem.drive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());

    }
}
