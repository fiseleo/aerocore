package dev.mcmod.aerocore.api.hydraulic;

/**
 * Sharing a pump's flow between the functions it feeds when it cannot supply all they ask for, as a load-independent
 * flow-sharing valve block does (LUDV, "lastdruckunabhängige Durchflussverteilung"; Findeisen and Helduser,
 * Ölhydraulik, Springer; Bosch Rexroth's LUDV control blocks such as the M7 and SX series): the compensators behind
 * the spools all see the same pressure, so when the pump saturates every function slows by the same factor whatever
 * its load, and the operator keeps the ratio of the movements (boom against arm). A plain load-sensing block without
 * it starves the heaviest load first instead (not modelled).
 *
 * <p>Requests and flows are m^3/s, not negative. A function whose load needs more than the relief or compensator
 * setting gets nothing: pass it a request of zero. A negative supply (a pump turning backwards) counts as none. The
 * result is continuous in the supply and in every request. The out arrays may be the requests array itself; the
 * methods that return a new array allocate it.
 */
public final class FlowShare {
    private FlowShare() {}

    /**
     * Shares the available flow among the requests into out: every request in full when their sum fits, otherwise
     * each scaled by available / sum (the ratios kept, the sum equal to the available flow). Returns the factor
     * (1 when everything fits, including when nothing is asked for).
     */
    public static double share(double availableM3PerS, double[] requestsM3PerS, double[] out) {
        double available = Math.max(0, Hydraulics.finite(availableM3PerS, "Available flow"));
        check(requestsM3PerS, out);
        return scale(available, requestsM3PerS, 0, out);
    }

    /** share into a new array. */
    public static double[] share(double availableM3PerS, double[] requestsM3PerS) {
        double[] out = new double[requestsM3PerS.length];
        share(availableM3PerS, requestsM3PerS, out);
        return out;
    }

    /**
     * Shares the available flow with priority into out: the first priorityCount requests are served in order, each in
     * full while the flow lasts (a priority valve, as for steering or brakes ahead of the working functions), and
     * what is left is shared among the rest as share does. Returns the factor of the shared rest.
     */
    public static double prioritised(double availableM3PerS, double[] requestsM3PerS, int priorityCount, double[] out) {
        double available = Math.max(0, Hydraulics.finite(availableM3PerS, "Available flow"));
        check(requestsM3PerS, out);
        if (priorityCount < 0 || priorityCount > requestsM3PerS.length) { throw new IllegalArgumentException("Priority count must be within the requests"); }
        for (int i = 0; i < priorityCount; i++) {
            double served = Math.min(requestsM3PerS[i], available);
            out[i] = served;
            available = Math.max(0, available - served);
        }
        return scale(available, requestsM3PerS, priorityCount, out);
    }

    /** prioritised into a new array. */
    public static double[] prioritised(double availableM3PerS, double[] requestsM3PerS, int priorityCount) {
        double[] out = new double[requestsM3PerS.length];
        prioritised(availableM3PerS, requestsM3PerS, priorityCount, out);
        return out;
    }

    private static double scale(double available, double[] requests, int from, double[] out) {
        double sum = 0;
        for (int i = from; i < requests.length; i++) { sum += requests[i]; }
        double factor = sum > available ? available / sum : 1;
        for (int i = from; i < requests.length; i++) { out[i] = requests[i] * factor; }
        return factor;
    }

    private static void check(double[] requests, double[] out) {
        if (out.length < requests.length) { throw new IllegalArgumentException("The out array is shorter than the requests"); }
        for (double r : requests) { Hydraulics.nonNegative(r, "Requested flow"); }
    }
}
