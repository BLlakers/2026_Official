package frc.robot.support;

/**
 * A triple record maintaining values suitable for instantiation of a {@link edu.wpi.first.math.controller.PIDController}
 * @param p Factor for "proportional" control
 * @param i Factor for "integral" control
 * @param d Factor for "derivative" control
 */
public record PIDSettings(double p, double i, double d) {
}
