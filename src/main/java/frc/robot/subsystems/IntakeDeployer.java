package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import static frc.robot.Constants.IntakeDeployerConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeDeployer extends SubsystemBase{
    private SparkFlex deployMotor;
    private State upState = new State(upSetPoint, 0.0);
    private State downState = new State(downSetPoint, 0.0);
    private SparkFlexConfig motorConfig;
    private static double mP = kP;
    private static double mI = kI;
    private static double mD = kD;
    private double intSetpointPosition;
    private double setpointIncrement = 0.005;

    private ArmFeedforward ff;

    public IntakeDeployer() {
        deployMotor = new SparkFlex(deployerCanId, MotorType.kBrushless);
        ff = new ArmFeedforward(0, kG, kV, kA);
        motorConfig = new SparkFlexConfig();
        motorConfig.closedLoop.pid(kP, kI, kD);
        motorConfig.closedLoop.outputRange(-.5, .5);
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        motorConfig.absoluteEncoder.inverted(true);
        motorConfig.inverted(true);
        deployMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Extend Increment", setpointIncrement);
        SmartDashboard.putNumber("Intake Deployer P", mP);
        SmartDashboard.putNumber("Intake Deployer I", mI);
        SmartDashboard.putNumber("Intake Deployer D", mD);
    }

    public void intExtendSetpointPosition() {
        intSetpointPosition = upState.position + setpointIncrement;
    }

    public void retract() {
        deployMotor.set(retractSpeed);
    }

    public void extend() {
        deployMotor.set(extendSpeed);
    }

    public void runToExtendedPosition() {
        if (intSetpointPosition + setpointIncrement <= downState.position)
            intSetpointPosition += setpointIncrement;
        else 
            intSetpointPosition = downState.position;
        double feedforward = ff.calculate(intSetpointPosition*2*Math.PI, downState.velocity);
        deployMotor.getClosedLoopController().setSetpoint(intSetpointPosition, ControlType.kPosition, 
                                                                    ClosedLoopSlot.kSlot0, feedforward);
        SmartDashboard.putNumber("Feed Forward Calculated",feedforward);
    }

    public void runToRetractedPosition() {
        double feedforward = ff.calculate(upState.position*2*Math.PI, upState.velocity);
        deployMotor.getClosedLoopController().setSetpoint(upSetPoint, ControlType.kPosition, 
                                                                    ClosedLoopSlot.kSlot0, feedforward);
    }

    public void stop() {
        deployMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double pVal = SmartDashboard.getNumber("Intake Deployer P", mP);
        double iVal = SmartDashboard.getNumber("Intake Deployer I", mI);
        double dVal = SmartDashboard.getNumber("Intake Deployer D", mD);
        setpointIncrement = SmartDashboard.getNumber("Extend Increment", setpointIncrement);

        if (pVal != mP || iVal != mI || dVal != mD) {
            mP = pVal;
            mI = iVal;
            mD = dVal;
            motorConfig.closedLoop.pid(mP, mI, mD);
        }
        SmartDashboard.putNumber("Deployer Position", deployMotor.getAbsoluteEncoder().getPosition());
    }
}
