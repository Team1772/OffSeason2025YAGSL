package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax motorLeft;
    private SparkMax motorRight;

    public IntakeSubsystem(){
        motorLeft = new SparkMax(51, MotorType.kBrushless);
        motorRight = new SparkMax(52, MotorType.kBrushless);
    }
}
