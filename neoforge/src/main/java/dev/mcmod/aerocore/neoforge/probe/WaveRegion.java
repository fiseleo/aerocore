package dev.mcmod.aerocore.neoforge.probe;

import dev.mcmod.aerocore.api.CompiledWaves;
import dev.mcmod.aerocore.api.LinearWave;
import dev.mcmod.aerocore.api.WaveField;
import net.minecraft.network.FriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * A circular sea area: free surfaces at stillWaterY (the level measured at the centre) inside it move with a prescribed
 * linear wave field (none for a calm sea), shared by every body, vanilla entity and the client renderer. Inside
 * radius - taper the field is the plain superposition; across the outer taper its amplitude falls smoothly to zero so
 * the surface meets the static vanilla water. The taper is a blending choice, not wave physics. A sea-state record
 * (JONSWAP parameters) is kept for display when the field was synthesised from a spectrum.
 */
public final class WaveRegion {
    public static final double MIN_RADIUS = 4, MAX_RADIUS = 128;
    public static final int MAX_COMPONENTS = 64;
    /** Free surfaces within this distance of stillWaterY belong to the region (vanilla source water is exact). */
    public static final double LEVEL_TOLERANCE = 0.01;
    /** Spectrum parameters of a synthesised sea: Hs (m), Tp (s), gamma, mean direction (rad), cos^n exponent. */
    public record Sea(double significantHeightM, double peakPeriodS, double gamma, double directionRadians, double spreading) {}
    public static final StreamCodec<FriendlyByteBuf, WaveRegion> STREAM_CODEC = StreamCodec.of(WaveRegion::write, WaveRegion::read);

    private final double centerX, centerZ, radiusM, stillWaterY;
    private final WaveField waves;
    private final CompiledWaves compiled;
    private final Sea sea;

    public WaveRegion(double centerX, double centerZ, double radiusM, double stillWaterY, WaveField waves) {
        this(centerX, centerZ, radiusM, stillWaterY, waves, null);
    }
    public WaveRegion(double centerX, double centerZ, double radiusM, double stillWaterY, WaveField waves, Sea sea) {
        if (!Double.isFinite(centerX) || !Double.isFinite(centerZ) || !Double.isFinite(stillWaterY)) {
            throw new IllegalArgumentException("Wave region position must be finite");
        }
        if (!Double.isFinite(radiusM) || radiusM < MIN_RADIUS || radiusM > MAX_RADIUS) {
            throw new IllegalArgumentException("Wave region radius must be in [4, 128] m");
        }
        Objects.requireNonNull(waves, "waves");
        if (waves.components().size() > MAX_COMPONENTS) { throw new IllegalArgumentException("At most 64 wave components"); }
        this.centerX = centerX; this.centerZ = centerZ; this.radiusM = radiusM; this.stillWaterY = stillWaterY;
        this.waves = waves; this.compiled = new CompiledWaves(waves); this.sea = sea;
    }

    public double centerX() { return centerX; }
    public double centerZ() { return centerZ; }
    public double radiusM() { return radiusM; }
    public double stillWaterY() { return stillWaterY; }
    public WaveField waves() { return waves; }
    public CompiledWaves compiled() { return compiled; }
    /** The spectrum this sea was synthesised from, or null for a single wave or a calm sea. */
    public Sea sea() { return sea; }

    /** Width of the outer amplitude ramp: a quarter of the radius, kept between 2 and 8 m. */
    public double taperM() { return Math.clamp(radiusM / 4, 2, 8); }

    /** Amplitude factor at a horizontal position: 1 in the core, smoothstep to 0 at the rim, 0 outside. */
    public double weight(double x, double z) {
        double r = Math.hypot(x - centerX, z - centerZ);
        if (r >= radiusM) { return 0; }
        double inner = radiusM - taperM();
        if (r <= inner) { return 1; }
        double s = (radiusM - r) / taperM();
        return s * s * (3 - 2 * s);
    }

    public boolean contains(double x, double z) { return Math.hypot(x - centerX, z - centerZ) < radiusM; }
    public boolean overlaps(WaveRegion other) { return Math.hypot(centerX - other.centerX, centerZ - other.centerZ) < radiusM + other.radiusM; }
    /** True for a free surface at the region's still level; other water bodies inside the circle stay calm. */
    public boolean ownsSurface(double surfaceY) { return Math.abs(surfaceY - stillWaterY) <= LEVEL_TOLERANCE; }
    /** The field with the local taper applied; the factor is taken at the given point and held over a small body. */
    public WaveField fieldAt(double x, double z) { return waves.scaled(weight(x, z)); }
    /** Instantaneous tapered elevation above the still level. */
    public double elevation(double x, double z, double t) {
        double w = weight(x, z);
        return w == 0 ? 0 : w * compiled.elevation(x, z, t);
    }

    @Override public boolean equals(Object o) {
        return o instanceof WaveRegion r && r.centerX == centerX && r.centerZ == centerZ && r.radiusM == radiusM
                && r.stillWaterY == stillWaterY && r.waves.equals(waves) && Objects.equals(r.sea, sea);
    }
    @Override public int hashCode() { return Objects.hash(centerX, centerZ, radiusM, stillWaterY, waves, sea); }
    @Override public String toString() { return "WaveRegion[" + centerX + ", " + centerZ + ", r " + radiusM + ", y " + stillWaterY + ", " + waves.components().size() + " components]"; }

    private static void write(FriendlyByteBuf buffer, WaveRegion region) {
        buffer.writeDouble(region.centerX).writeDouble(region.centerZ).writeDouble(region.radiusM).writeDouble(region.stillWaterY);
        buffer.writeVarInt(region.waves.components().size());
        for (LinearWave wave : region.waves.components()) {
            buffer.writeDouble(wave.amplitudeM()).writeDouble(wave.wavelengthM()).writeDouble(wave.directionRadians())
                    .writeDouble(wave.depthM()).writeDouble(wave.phaseRadians());
        }
        buffer.writeBoolean(region.sea != null);
        if (region.sea != null) {
            buffer.writeDouble(region.sea.significantHeightM()).writeDouble(region.sea.peakPeriodS()).writeDouble(region.sea.gamma())
                    .writeDouble(region.sea.directionRadians()).writeDouble(region.sea.spreading());
        }
    }

    private static WaveRegion read(FriendlyByteBuf buffer) {
        double x = buffer.readDouble(), z = buffer.readDouble(), radius = buffer.readDouble(), still = buffer.readDouble();
        int count = buffer.readVarInt();
        if (count < 0 || count > MAX_COMPONENTS) { throw new IllegalArgumentException("Too many wave components"); }
        List<LinearWave> waves = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            waves.add(new LinearWave(buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble()));
        }
        Sea sea = buffer.readBoolean() ? new Sea(buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble(), buffer.readDouble()) : null;
        return new WaveRegion(x, z, radius, still, new WaveField(waves), sea);
    }
}
