package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sim.SparkMaxEncoderWrapper;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;

import java.util.List;

public class ArmS extends SubsystemBase implements Loggable {
    @Log
    public final Field2d VISUALIZER = new Field2d();

    public ArmS() {
        initExtender();
        initPivot();

        var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
        VISUALIZER.getObject("2_target").setPose(new Pose2d(
            pivotPose.getTranslation().plus(new Translation2d(0, MIN_ARM_LENGTH)),
            pivotPose.getRotation()
        ));
    }

    public void periodic() {
        
        pivotPeriodic();
        updateVisualizer();
    }

    public void updateVisualizer() {
        var pivotPose = new Pose2d(ARM_PIVOT_TRANSLATION, getAngle());
        VISUALIZER.getObject("0_Pivot").setPose(pivotPose);
        VISUALIZER.getObject("1_Arm").setPoses(List.of(pivotPose, pivotPose.transformBy(
            new Transform2d(new Translation2d(getLengthMeters(), 0), new Rotation2d())
        )));
    }

    public void simulationPeriodic() {
        extendSimulationPeriodic();
        pivotSimulationPeriodic();
        
    }

    // region extend
    private final Subsystem extendS = new Subsystem() {};
    private final CANSparkMax m_extendMotor = new CANSparkMax(EXTEND_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_extendEncoderWrapper = new SparkMaxEncoderWrapper(m_extendMotor);
    private final LinearSystem<N2, N1, N1> m_extendPlant =
        LinearSystemId.identifyPositionSystem(
            12 / (Units.inchesToMeters(81.2)), 0.01);
    private final TiltedElevatorSim m_extendSim = new TiltedElevatorSim(
        m_extendPlant,
        DCMotor.getNEO(1),
        EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION,
        EXTEND_DRUM_RADIUS,
        MIN_ARM_LENGTH, MAX_ARM_LENGTH, true);
    
    private final LinearPlantInversionFeedforward<N2,N1,N1> m_extendFeedforward
        = new LinearPlantInversionFeedforward<>(m_extendPlant, 0.02);

    private final ProfiledPIDController m_extendController =
        new ProfiledPIDController(5,0,0,
            new Constraints(2, 4)
        );

    public void initExtender() {
        m_extendMotor.getEncoder().setPositionConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION);
        m_extendMotor.getEncoder().setVelocityConversionFactor(EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * EXTEND_METERS_PER_DRUM_ROTATION / 60);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_LENGTH);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_LENGTH);
        m_extendEncoderWrapper.setPosition(MIN_ARM_LENGTH);
        m_extendController.reset(MIN_ARM_LENGTH);
        m_extendSim.setState(VecBuilder.fill(MIN_ARM_LENGTH, 0));
    }

    public void extendPeriodic(){}

    public void extendSimulationPeriodic() {
        m_extendSim.setAngleFromHorizontal(getAngle().getRadians());
        m_extendSim.setInputVoltage(m_extendMotor.getAppliedOutput());
        m_extendSim.update(0.02);
        m_extendEncoderWrapper.setSimPosition(m_extendSim.getPositionMeters());
        m_extendEncoderWrapper.setSimVelocity(m_extendSim.getVelocityMetersPerSecond());
    }

    public void setExtendVolts(double volts) {
        m_extendMotor.setVoltage(volts);
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

    public void setExtendVelocity(double velocityMetersPerSecond) {
        m_extendMotor.setVoltage(
            m_extendFeedforward.calculate(
                VecBuilder.fill(0, getExtendVelocity()),
                VecBuilder.fill(0, velocityMetersPerSecond)
            ).get(0,0)
            + ARM_EXTEND_KG_VERTICAL * Math.sin(getAngle().getRadians())
        );
    }

    public void setExtendLength(double lengthMeters) {
        setExtendVelocity(
            m_extendController.calculate(getLengthMeters(), lengthMeters)
            +m_extendController.getSetpoint().velocity
        );
    }

    public Command extendC() {
        return extendS.run(()->{setExtendVolts(3);})
            .finallyDo((interrupted)->setExtendVolts(0));
    }

    public Command retractC() {
        return extendS.run(()->setExtendVolts(-3))
        .finallyDo((interrupted)->setExtendVolts(0));
    }

    // endregion

    // region pivot
    private final Subsystem pivotS = new Subsystem() {};

    private final CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_pivotEncoderWrapper = new SparkMaxEncoderWrapper(m_pivotMotor);

    private LinearSystem<N2, N1, N1> m_pivotPlant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(1),  1.0 / 3.0 * ARM_MASS_KG * MIN_ARM_LENGTH * MIN_ARM_LENGTH
        , 1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION);

    private DCMotor m_pivotGearbox = DCMotor.getNEO(1);
    private final VariableLengthArmSim m_pivotSim = new VariableLengthArmSim(
        m_pivotPlant,
        DCMotor.getNEO(1),
        1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION,
        1.0 / 3.0 * ARM_MASS_KG * MIN_ARM_LENGTH * MIN_ARM_LENGTH,
        MIN_ARM_LENGTH,
        MIN_ARM_ANGLE,
        MAX_ARM_ANGLE,
        ARM_MASS_KG,
        true
    );

    private LinearPlantInversionFeedforward<N2, N1, N1> m_pivotFeedForward
        = new LinearPlantInversionFeedforward<>(m_pivotPlant, 0.02);

    private ProfiledPIDController m_pivotController = new ProfiledPIDController(
        5, 0, 0, new Constraints(4,4));

    private double armStartAngle = 0;

    private void initPivot() {
        m_pivotMotor.getEncoder().setPositionConversionFactor(ARM_ROTATIONS_PER_MOTOR_ROTATION);
        m_pivotMotor.getEncoder().setVelocityConversionFactor(ARM_ROTATIONS_PER_MOTOR_ROTATION / 60);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_ARM_ANGLE);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) MIN_ARM_ANGLE);

        m_pivotEncoderWrapper.setPosition(armStartAngle);
        m_pivotSim.setState(VecBuilder.fill(armStartAngle,0));

        m_pivotController.reset(armStartAngle);
        m_pivotController.enableContinuousInput(-Math.PI, Math.PI);
        m_pivotController.setTolerance(0.05, 0.05);

        pivotS.setDefaultCommand(followTargetC());
    }

    private void pivotPeriodic() {
        updatePivotPlant();
        m_pivotFeedForward = new LinearPlantInversionFeedforward<>(m_pivotPlant, 0.02);
    }
    private void pivotSimulationPeriodic() {
        m_pivotSim.setCGRadius(getLengthMeters() / 2);
        m_pivotSim.setMOI(getPivotMOI());
        m_pivotSim.setInputVoltage(m_pivotMotor.getAppliedOutput());
        m_pivotSim.update(0.02);
        m_pivotEncoderWrapper.setSimPosition(m_pivotSim.getAngleRads());
        m_pivotEncoderWrapper.setSimVelocity(m_pivotSim.getVelocityRadPerSec());
    }

    public void setPivotVolts(double volts) {
        m_pivotMotor.setVoltage(volts);
    }

    @Log(methodName = "getRadians")
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_pivotEncoderWrapper.getPosition());
    }

    @Log
    public double getPivotVelocity() {
        return m_pivotEncoderWrapper.getVelocity();
    }

    public double getPivotMOI() {
        return 1.0 / 3.0 * ARM_MASS_KG * getLengthMeters() * getLengthMeters();
    }

    public void updatePivotPlant() {
        m_pivotPlant.getA().set(1, 1, 
          -1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION * 1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION
          * m_pivotGearbox.KtNMPerAmp
          / (m_pivotGearbox.KvRadPerSecPerVolt * m_pivotGearbox.rOhms * getPivotMOI()));
        m_pivotPlant.getB().set(1, 0, 
          1.0/ARM_ROTATIONS_PER_MOTOR_ROTATION * m_pivotGearbox.KtNMPerAmp / (m_pivotGearbox.rOhms * getPivotMOI()));
    }

    public void setPivotVelocity(double velocityRadPerSec) {
        setPivotVolts(m_pivotFeedForward.calculate(VecBuilder.fill(0, getPivotVelocity()), VecBuilder.fill(0, velocityRadPerSec)).get(0,0) + (getPivotkG() * getAngle().getCos()));
    }

    public void setPivotAngle(double targetAngle) {
        SmartDashboard.putNumber("armRequestAngle", targetAngle);
        var outputVelocity = m_pivotController.calculate(
            getAngle().getRadians(),
            targetAngle
        );
        SmartDashboard.putNumber("armError", m_pivotController.getPositionError());
        SmartDashboard.putNumber("armRequestVel", outputVelocity + m_pivotController.getSetpoint().velocity);
        setPivotVelocity(outputVelocity + m_pivotController.getSetpoint().velocity);
    }

    @Log
    public double getPivotkG() {
        double minkG = ARM_PIVOT_KG_MIN_EXTEND;
        double maxkG = ARM_PIVOT_KG_MAX_EXTEND;

        double result = minkG;
        double s = (getLengthMeters() - MIN_ARM_LENGTH) / (MAX_ARM_LENGTH - MIN_ARM_LENGTH);
        result += s * (maxkG - minkG);
        return result;
    }

    public Command holdC() {
        return pivotS.run(()->setPivotAngle(0));
    }

    public Command counterClockwiseC() {
        return pivotS.run(()->setPivotVelocity(0.5))
        .finallyDo((interrupted)->setPivotVolts(0));
    }

    public Command clockwiseC() {
        return pivotS.run(()->setPivotVelocity(-0.5))
        .finallyDo((interrupted)->setPivotVolts(0));
    }
    // endregion

    // region factories
    public Command followTargetC() {
        return Commands.run(()->{
            var offset = VISUALIZER.getObject("2_target").getPose().getTranslation();
            offset = new Translation2d(offset.getX(), offset.getY() - Units.inchesToMeters(25));
            setPivotAngle(offset.getAngle().getRadians());
            setExtendLength(offset.getNorm());
        },
        extendS, pivotS);
    }

    // endregion

}
