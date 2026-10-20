package dev.mcmod.aerocore.api.wheel.kinematics;

/** A suspension's links bind (0.33): no position of the upright holds them all at a compression (m). */
public final class LinkageLockException extends RuntimeException {
    private final double compressionM;

    public LinkageLockException(double compressionM, String why) {
        super(String.format(java.util.Locale.ROOT, "The suspension's links bind at %.3f m: %s", compressionM, why));
        this.compressionM = compressionM;
    }

    /** Where they bind (m of compression; inside the solver, of rise from the design position). */
    public double compressionM() { return compressionM; }
}
