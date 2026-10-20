package dev.mcmod.aerocore.api;

/**
 * The textbook torque-speed envelope of a field-weakened electric machine: constant torque up to the base speed where
 * torque times speed reaches the rated power, constant power above it, nothing beyond the top speed. Braking (torque
 * against the rotation) is limited by the power the machine can absorb, not by its top speed.
 */
public record ElectricMotor(double maxTorqueNm, double maxPowerW, double maxSpeedRpm) {
    public ElectricMotor {
        if (!(maxTorqueNm > 0) || !(maxPowerW > 0) || !(maxSpeedRpm > 0)) { throw new IllegalArgumentException("Motor torque, power and speed must be positive"); }
    }

    /** Torque magnitude available at a speed (rad/s) in the direction of rotation (driving), N m. */
    public double available(double speed) {
        double w = Math.abs(speed), top = maxSpeedRpm * Math.PI / 30;
        if (w >= top) { return 0; }
        return Math.min(maxTorqueNm, maxPowerW / Math.max(w, 1e-9));
    }

    /** Torque on the rotor (N m) for a command -1..1 (negative drives in reverse) at a rotor speed (rad/s). */
    public double torque(double command, double speed) {
        double u = Math.clamp(command, -1, 1);
        if (u == 0) { return 0; }
        boolean driving = speed == 0 || Math.signum(u) == Math.signum(speed);
        double magnitude = driving ? available(speed) : Math.min(maxTorqueNm, maxPowerW / Math.max(Math.abs(speed), 1e-9));
        return u * magnitude;
    }

    /** Electrical power (W) for a shaft power at an efficiency: drawn when driving, regenerated when braking. */
    public static double electricalPower(double shaftPowerW, double efficiency) { return shaftPowerW >= 0 ? shaftPowerW / efficiency : shaftPowerW * efficiency; }
}
