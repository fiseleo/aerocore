package dev.mcmod.aerocore.api.vehicle;

import java.util.List;
import java.util.stream.Collectors;

/** A design with INVALID values was compiled; issues lists them (and any EXTRAPOLATION found with them). */
public final class DesignException extends IllegalArgumentException {
    private final List<DesignIssue> issues;

    public DesignException(List<DesignIssue> issues) {
        super(issues.stream().filter(i -> i.kind() == DesignIssue.Kind.INVALID).map(DesignIssue::message).collect(Collectors.joining("; ")));
        this.issues = List.copyOf(issues);
    }

    public List<DesignIssue> issues() { return issues; }

    /** Throws when any issue is INVALID. */
    static void throwIfInvalid(List<DesignIssue> issues) {
        if (issues.stream().anyMatch(i -> i.kind() == DesignIssue.Kind.INVALID)) { throw new DesignException(issues); }
    }
}
