using System;


namespace rt
{
    public class Ellipsoid : Geometry
    {
        private Vector Center { get; }
        private Vector SemiAxesLength { get; }
        private double Radius { get; }
        
        
        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Material material, Color color) : base(material, color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Color color) : base(color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public override Intersection GetIntersection(Line line, double minDist, double maxDist)
        {
            // Ray origin & (unit) direction — Line normalizes Dx in ctor
            var O = line.X0;
            var D = line.Dx;

            // Axis lengths (semi-axes scaled by Radius)
            double a = SemiAxesLength.X * Radius;
            double b = SemiAxesLength.Y * Radius;
            double c = SemiAxesLength.Z * Radius;

            // Transform ray to unit-sphere space
            var OC = O - Center;
            var Ot = new Vector(OC.X / a, OC.Y / b, OC.Z / c);
            var Dt = new Vector(D.X / a, D.Y / b, D.Z / c);

            // Quadratic for |Ot + t*Dt|^2 = 1  ->  A t^2 + B t + C = 0
            double A = Dt * Dt;
            double B = 2.0 * (Ot * Dt);
            double Cq = (Ot * Ot) - 1.0;

            double disc = B * B - 4.0 * A * Cq;
            if (disc < 0.0 || A == 0.0) return Intersection.NONE;

            double sqrtDisc = Math.Sqrt(disc);
            double inv2A = 0.5 / A;

            double t1 = (-B - sqrtDisc) * inv2A; // near
            double t2 = (-B + sqrtDisc) * inv2A; // far
            if (t1 > t2) { var tmp = t1; t1 = t2; t2 = tmp; }

            // pick the closest valid t in [minDist, maxDist]
            const double eps = 1e-6;
            double t = double.NaN;
            if (t1 >= Math.Max(minDist, eps) && t1 <= maxDist) t = t1;
            else if (t2 >= Math.Max(minDist, eps) && t2 <= maxDist) t = t2;
            else return Intersection.NONE;

            // Hit point (world space)
            var P = line.CoordinateToPosition(t);

            // Ellipsoid normal (world space): grad f = ((x-cx)/a^2, (y-cy)/b^2, (z-cz)/c^2)
            var N = new Vector(
                (P.X - Center.X) / (a * a),
                (P.Y - Center.Y) / (b * b),
                (P.Z - Center.Z) / (c * c)
            ).Normalize();

            // Build intersection (valid & visible)
            return new Intersection(
                valid: true,
                visible: true,
                geometry: this,
                line: line,
                t: t,
                normal: N,
                material: this.Material,
                color: this.Color
            );
        }

    }
}
