package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sim.SparkMaxEncoderWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;

import java.util.List;

public class ArmS extends SubsystemBase implements Loggable {
    @Log
    public final Field2d VISUALIZER = new Field2d();

    /* EXTENDER */
    private final CANSparkMax m_extendMotor = new CANSparkMax(EXTEND_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_extendEncoderWrapper = new SparkMaxEncoderWrapper(m_extendMotor);
    private final LinearSystem<N2, N1, N1> m_extendPlant =
        LinearSystemId.identifyPositionSystem(
            12 / (Units.inchesToMeters(81.2)), 0.01);
    private final ElevatorSim m_extendSim = new ElevatorSim(
        m_extendPlant,
        DCMotor.getNEO(1),
        EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION,
        EXTEND_DRUM_RADIUS,
        MIN_ARM_LENGTH, MAX_ARM_LENGTH, false);

    /* PIVOT */

    private final CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_pivotEncoderWrapper = new SparkMaxEncoderWrapper(m_pivotMotor);
    private final VariableLengthArmSim m_pivotSim = new VariableLengthArmSim(
        DCMotor.getNEO(1),
        1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION,
        VariableLengthArmSim.estimateMOI(MIN_ARM_LENGTH, ARM_MASS_KG),
        MIN_ARM_LENGTH,
        MIN_ARM_ANGLE,
        MAX_ARM_ANGLE,
        ARM_MASS_KG,
        true
    );

    public ArmS() {
        m_extendMotor.getEncoder().setPositionConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION);
        m_extendMotor.getEncoder().setVelocityConversionFactor(
            EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION / 60
        );
        m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_LENGTH);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_LENGTH);
        m_extendEncoderWrapper.setPosition(MIN_ARM_LENGTH);
        m_pivotMotor.getEncoder().setPositionConversionFactor(ARM_ROTATIONS_PER_MOTOR_ROTATION);
        m_pivotMotor.getEncoder().setVelocityConversionFactor(ARM_ROTATIONS_PER_MOTOR_ROTATION / 60);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_ANGLE);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_ANGLE);
    }

    public void periodic() {
        var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
        VISUALIZER.getObject("0_Pivot").setPose(pivotPose);
        VISUALIZER.getObject("1_Arm").setPoses(List.of(pivotPose, pivotPose.transformBy(
            new Transform2d(new Translation2d(getLengthMeters(), 0), new Rotation2d())
        )));
    }

    public void simulationPeriodic() {
            m_extendSim.setInputVoltage(m_extendMotor.getAppliedOutput());
            m_extendSim.update(0.02);
            m_extendEncoderWrapper.setSimPosition(m_extendSim.getPositionMeters());
            m_extendEncoderWrapper.setSimVelocity(m_extendSim.getVelocityMetersPerSecond());

            m_pivotSim.setLength(getLengthMeters());
            m_pivotSim.setInputVoltage(m_pivotMotor.getAppliedOutput());
            m_pivotSim.update(0.02);
            m_pivotEncoderWrapper.setSimPosition(m_pivotSim.getAngleRads());
            m_pivotEncoderWrapper.setSimVelocity(m_pivotSim.getVelocityRadPerSec());
    }
    /**
     * Returns the distance from the pivot to the wrist joint in meters.
     * @return
     */
    @Log
    public double getLengthMeters() {
        return m_extendEncoderWrapper.getPosition();
    }

    @Log
    public double getExtendVelocity(){
        return m_extendEncoderWrapper.getVelocity();
    }

    @Log(methodName = "getRadians")
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_pivotEncoderWrapper.getPosition());
    }

    @Log
    public double getPivotVelocity() {
        return m_pivotEncoderWrapper.getVelocity();
    }

    public void setExtendVolts(double volts) {
        m_extendMotor.setVoltage(volts);
    }

    public void setPivotVolts(double volts) {
        m_pivotMotor.setVoltage(volts);
    }

    @Log
    public double getArmkG() {
        double minkG = 1.414;
        double maxkG = 2.872;

        double result = 1.414;
        double s = (getLengthMeters() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
        result += s * (2.872 - 1.414);
        return result;
    }

    /** kG cos (pi/4) = 1
     * kG = 1/cos(pi/4)
     * = 2/sqrt2
     * =sqrt2
     * 
     * kG at full retract = 1.414
     * kG at full extend:
     * kG cos (10deg) = 2.828
     * kG = 2.828 / cos (10deg)
     * =2.872
     */
    public Command holdC() {
        return run(()->
        setPivotVolts(
            getArmkG() * getAngle().getCos()));
    }

    public Command extendC() {
        return run(()->{setExtendVolts(3);})
            .finallyDo((interrupted)->setExtendVolts(0));
    }

    public Command retractC() {
        return run(()->setExtendVolts(-3))
        .finallyDo((interrupted)->setExtendVolts(0));
    }

    public Command counterClockwiseC() {
        return run(()->setPivotVolts(12))
        .finallyDo((interrupted)->setPivotVolts(0));
    }

    public Command clockwiseC() {
        return run(()->setPivotVolts(-12))
        .finallyDo((interrupted)->setPivotVolts(0));
    }
}
