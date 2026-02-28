package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FeederConstants.*;
import static frc.robot.Constants.IntakeDeployerConstants.kV;
import static frc.robot.Constants.*;

public class Feeder extends SubsystemBase{
    

    private SparkMax motor1;
    //private SparkMax motor2;

    private SparkFlexConfig motor1Config;
    //private SparkFlexConfig motor2Config;

    private double fP;
    private double fI;
    private double fD;
    private double fFF;

    public Feeder() { 
        fP = kP;
        fI = kI;
        fD = kD;
        fFF = kFF;


        motor1 = new SparkMax(motor1CanId, MotorType.kBrushless);
        //motor2 = new SparkMax(motor2CanId, MotorType.kBrushless);


        SmartDashboard.putNumber("FeederP", fP);
        SmartDashboard.putNumber("FeederI", fI);
        SmartDashboard.putNumber("FeederD", fD);
        SmartDashboard.putNumber("FeederFF", fFF);

        motor1Config = createConfigurationForVelocity(true);
        //motor2Config = createConfigurationForVelocity(true);
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void feed() {
        motor1.set(0.3);
        //motor2.set(0.3);
    }

    public void feedAtConstantVelocity() {
        motor1.getClosedLoopController().setSetpoint(setPoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, fFF);
        //motor2.getClosedLoopController().setSetpoint(setPoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, fFF);
    }

    public void stop() {
        motor1.stopMotor();
        //motor2.stopMotor();
    }

    private SparkFlexConfig createConfigurationForVelocity(boolean isInverted){
        var motorConfig = new SparkFlexConfig();
        motorConfig.closedLoop.pid(kP, kI, kD);
        motorConfig.closedLoop.maxOutput(1.0);
        motorConfig.closedLoop.minOutput(-1.0);
        motorConfig.inverted(isInverted);
        return motorConfig;
    }

    @Override
    public void periodic(){
        double pVal = SmartDashboard.getNumber("FeederP", fP);
        double iVal = SmartDashboard.getNumber("FeederI", fI);
        double dVal = SmartDashboard.getNumber("FeederD", fD);
        fFF = SmartDashboard.getNumber("FeederFF", fFF);
        if (pVal != fP || iVal != fI || dVal != fD) {
            motor1Config.closedLoop.pid(pVal, iVal, dVal);
            //motor2Config.closedLoop.pid(pVal, iVal, dVal);
            motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            //motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            fP = pVal;
            fI = iVal;
            fD = dVal;

        }
        SmartDashboard.putNumber("Feeder Motor1 Velocity", motor1.getEncoder().getVelocity());
        //SmartDashboard.putNumber("Feeder Motor2 Velocity", motor2.getEncoder().getVelocity());

    }
}
