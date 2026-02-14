package frc.robot.support.sparkmax;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.system.plant.DCMotor;

public class TeamSparkFlexSimImpl extends TeamSparkFlexImpl {

    private final SparkFlexSim sparkFlexSim;

    public TeamSparkFlexSimImpl(int channel, SparkLowLevel.MotorType type) {
        super(channel, type);
        this.sparkFlexSim = new SparkFlexSim(this.sparkFlex, DCMotor.getNeoVortex(1));
    }

    @Override
    public double getAppliedOutput() {
        return this.sparkFlexSim.getAppliedOutput();
    }

    @Override
    public double getPosition() {
        return this.sparkFlexSim.getRelativeEncoderSim().getPosition();
    }

    @Override
    public double getVelocity() {
        return this.sparkFlexSim.getRelativeEncoderSim().getVelocity();
    }
}
