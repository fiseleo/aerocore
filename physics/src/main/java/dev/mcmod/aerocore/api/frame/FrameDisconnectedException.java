package dev.mcmod.aerocore.api.frame;

/** A frame falls apart (0.33): a node joined to no member, or to a piece the rest of the frame does not reach. */
public final class FrameDisconnectedException extends IllegalStateException {
    private final int node;

    public FrameDisconnectedException(int node) {
        super("節點 " + node + " 沒接上 (node " + node + " is not joined to the rest of the frame)");
        this.node = node;
    }

    /** The node (its index in the Frame). */
    public int node() { return node; }
}
