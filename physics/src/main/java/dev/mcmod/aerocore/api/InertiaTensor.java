package dev.mcmod.aerocore.api;

/** Symmetric positive-definite inertia about the COM, in body coordinates, kg m^2. */
public record InertiaTensor(double xx, double yy, double zz, double xy, double xz, double yz) {
    public InertiaTensor {
        double determinant = xx*(yy*zz-yz*yz)-xy*(xy*zz-yz*xz)+xz*(xy*yz-yy*xz);
        if (!Double.isFinite(xx+yy+zz+xy+xz+yz) || xx <= 0 || xx*yy-xy*xy <= 0
                || !Double.isFinite(determinant) || determinant <= 0) {
            throw new IllegalArgumentException("Inertia must be finite and positive definite");
        }
    }
    public static InertiaTensor box(double mass, Vec3d size) {
        Checks.positive(mass, "Mass");
        Checks.positive(size.x(), "Width"); Checks.positive(size.y(), "Height"); Checks.positive(size.z(), "Depth");
        return new InertiaTensor(mass*(size.y()*size.y()+size.z()*size.z())/12,
                mass*(size.x()*size.x()+size.z()*size.z())/12, mass*(size.x()*size.x()+size.y()*size.y())/12, 0, 0, 0);
    }
    public Vec3d apply(Vec3d v) {
        return new Vec3d(xx*v.x()+xy*v.y()+xz*v.z(), xy*v.x()+yy*v.y()+yz*v.z(), xz*v.x()+yz*v.y()+zz*v.z());
    }
    public Vec3d solve(Vec3d v) {
        double a=yy*zz-yz*yz, b=xz*yz-xy*zz, c=xy*yz-xz*yy;
        double d=xx*zz-xz*xz, e=xy*xz-xx*yz, f=xx*yy-xy*xy;
        return new Vec3d(a*v.x()+b*v.y()+c*v.z(), b*v.x()+d*v.y()+e*v.z(), c*v.x()+e*v.y()+f*v.z())
                .scale(1/(xx*a+xy*b+xz*c));
    }
    public InertiaTensor add(InertiaTensor b) {
        return new InertiaTensor(xx+b.xx, yy+b.yy, zz+b.zz, xy+b.xy, xz+b.xz, yz+b.yz);
    }
    public InertiaTensor shifted(double mass, Vec3d r) {
        Checks.nonnegative(mass, "Mass");
        return new InertiaTensor(xx+mass*(r.y()*r.y()+r.z()*r.z()), yy+mass*(r.x()*r.x()+r.z()*r.z()),
                zz+mass*(r.x()*r.x()+r.y()*r.y()), xy-mass*r.x()*r.y(), xz-mass*r.x()*r.z(), yz-mass*r.y()*r.z());
    }
    public Vec3d applyWorld(Orientation q, Vec3d v) { return q.rotate(apply(q.inverse().rotate(v))); }
    public Vec3d solveWorld(Orientation q, Vec3d v) { return q.rotate(solve(q.inverse().rotate(v))); }
}
