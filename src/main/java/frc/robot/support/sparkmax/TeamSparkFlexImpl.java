package frc.robot.support.sparkmax;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;

public class TeamSparkFlexImpl implements TeamSpark {

    protected final SparkFlex sparkFlex;

    public TeamSparkFlexImpl(int channel, SparkLowLevel.MotorType type) {
        this.sparkFlex = new SparkFlex(channel, type);
    }

    @Override
    public REVLibError configure(
            SparkBaseConfig config, SparkBase.ResetMode resetMode, SparkBase.PersistMode persistMode) {
        return this.sparkFlex.configure(config, resetMode, persistMode);
    }

    @Override
    public void set(double speed) {
        this.sparkFlex.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        this.sparkFlex.setVoltage(voltage);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.sparkFlex.setInverted(inverted);
    }

    @Override
    public void stopMotor() {
        this.sparkFlex.stopMotor();
    }

    @Override
    public int getDeviceId() {
        return this.sparkFlex.getDeviceId();
    }

    @Override
    public RelativeEncoder getEncoder() {
        return this.sparkFlex.getEncoder();
    }

    @Override
    public RelativeEncoder getAlternateEncoder() {
        // SparkFlex uses getExternalEncoder() instead of getAlternateEncoder()
        return this.sparkFlex.getExternalEncoder();
    }

    @Override
    public double getAppliedOutput() {
        return this.sparkFlex.getAppliedOutput();
    }

    @Override
    public double getPosition() {
        return this.sparkFlex.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return this.sparkFlex.getEncoder().getVelocity();
    }
}
