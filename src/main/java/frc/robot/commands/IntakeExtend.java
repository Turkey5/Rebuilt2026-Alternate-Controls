package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeployer;

public class IntakeExtend extends Command{

    IntakeDeployer intakeDeployer;
    
    public IntakeExtend(IntakeDeployer intakeDeployer) {
        this.intakeDeployer = intakeDeployer;
        this.addRequirements(intakeDeployer);
    }

    @Override
    public void initialize() {
        intakeDeployer.intExtendSetpointPosition();
    }

    @Override
    public void execute() {
        intakeDeployer.runToExtendedPosition();
    }

    @Override
    public void end(boolean isInterupted) {
        intakeDeployer.stop();
    }
}
