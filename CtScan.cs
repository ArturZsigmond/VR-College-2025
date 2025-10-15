using System;
using System.IO;
using System.Text.RegularExpressions;

namespace rt;

public class CtScan: Geometry
{
    private readonly Vector _position;
    private readonly double _scale;
    private readonly ColorMap _colorMap;
    private readonly byte[] _data;

    private readonly int[] _resolution = new int[3];
    private readonly double[] _thickness = new double[3];
    private readonly Vector _v0;
    private readonly Vector _v1;

    public CtScan(string datFile, string rawFile, Vector position, double scale, ColorMap colorMap) : base(Color.NONE)
    {
        _position = position;
        _scale = scale;
        _colorMap = colorMap;

        var lines = File.ReadLines(datFile);
        foreach (var line in lines)
        {
            var kv = Regex.Replace(line, "[:\\t ]+", ":").Split(":");
            if (kv[0] == "Resolution")
            {
                _resolution[0] = Convert.ToInt32(kv[1]);
                _resolution[1] = Convert.ToInt32(kv[2]);
                _resolution[2] = Convert.ToInt32(kv[3]);
            } else if (kv[0] == "SliceThickness")
            {
                _thickness[0] = Convert.ToDouble(kv[1]);
                _thickness[1] = Convert.ToDouble(kv[2]);
                _thickness[2] = Convert.ToDouble(kv[3]);
            }
        }

        _v0 = position;
        _v1 = position + new Vector(_resolution[0]*_thickness[0]*scale, _resolution[1]*_thickness[1]*scale, _resolution[2]*_thickness[2]*scale);

        var len = _resolution[0] * _resolution[1] * _resolution[2];
        _data = new byte[len];
        using FileStream f = new FileStream(rawFile, FileMode.Open, FileAccess.Read);
        if (f.Read(_data, 0, len) != len)
        {
            throw new InvalidDataException($"Failed to read the {len}-byte raw data");
        }
    }
    
    private ushort Value(int x, int y, int z)
    {
        if (x < 0 || y < 0 || z < 0 || x >= _resolution[0] || y >= _resolution[1] || z >= _resolution[2])
        {
            return 0;
        }

        return _data[z * _resolution[1] * _resolution[0] + y * _resolution[0] + x];
    }

    public override Intersection GetIntersection(Line line, double minDist, double maxDist)
    {
        // Axis-aligned bounding box for the volume
        var bmin = new Vector(
            Math.Min(_v0.X, _v1.X),
            Math.Min(_v0.Y, _v1.Y),
            Math.Min(_v0.Z, _v1.Z));
        var bmax = new Vector(
            Math.Max(_v0.X, _v1.X),
            Math.Max(_v0.Y, _v1.Y),
            Math.Max(_v0.Z, _v1.Z));

        var O = line.X0;      // origin
        var D = line.Dx;      // (unit) direction

        // --- Ray–AABB intersection (slab method) ---
        // Handle zero components robustly.
        double tmin = double.NegativeInfinity, tmax = double.PositiveInfinity;

        void slab(double o, double d, double minv, double maxv)
        {
            if (Math.Abs(d) < 1e-12)
            {
                // Ray is parallel to the slab: must be inside the slab
                if (o < minv || o > maxv) { tmin = 1; tmax = 0; } // force no hit
                return;
            }
            double t1 = (minv - o) / d;
            double t2 = (maxv - o) / d;
            if (t1 > t2) { var tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
        }

        slab(O.X, D.X, bmin.X, bmax.X);
        slab(O.Y, D.Y, bmin.Y, bmax.Y);
        slab(O.Z, D.Z, bmin.Z, bmax.Z);

        if (tmax < tmin) return Intersection.NONE;

        // Clamp to requested interval
        const double eps = 1e-6;
        double tEnter = Math.Max(tmin, Math.Max(minDist, eps));
        double tExit = Math.Min(tmax, maxDist);
        if (tExit < tEnter) return Intersection.NONE;

        // --- March through the volume and look for a non-empty voxel ---
        // Step size: a fraction of the smallest voxel thickness in world units.
        double cellX = _thickness[0] * _scale;
        double cellY = _thickness[1] * _scale;
        double cellZ = _thickness[2] * _scale;
        double baseStep = Math.Max(1e-4, Math.Min(cellX, Math.Min(cellY, cellZ)));
        double step = 0.5 * baseStep; // half-voxel for smoother capture

        // Simple threshold: treat value==0 as empty, >0 as occupied.
        // (You can replace with a configurable iso-threshold later.)
        for (double t = tEnter; t <= tExit; t += step)
        {
            var p = line.CoordinateToPosition(t);
            var idx = GetIndexes(p);

            // quick reject if outside resolution bounds (can happen at boundaries)
            if (idx[0] < 0 || idx[1] < 0 || idx[2] < 0 ||
                idx[0] >= _resolution[0] || idx[1] >= _resolution[1] || idx[2] >= _resolution[2])
                continue;

            var v = Value(idx[0], idx[1], idx[2]);
            if (v > 0)
            {
                // We have density -> surface hit at this sample.
                // Color from transfer function, normal from central differences.
                var color = GetColor(p);
                var normal = GetNormal(p);

                return new Intersection(
                    valid: true,
                    visible: true,
                    geometry: this,
                    line: line,
                    t: t,
                    normal: normal,
                    material: this.Material,
                    color: color
                );
            }
        }

        return Intersection.NONE;
    }


    private int[] GetIndexes(Vector v)
    {
        return new []{
            (int)Math.Floor((v.X - _position.X) / _thickness[0] / _scale), 
            (int)Math.Floor((v.Y - _position.Y) / _thickness[1] / _scale),
            (int)Math.Floor((v.Z - _position.Z) / _thickness[2] / _scale)};
    }
    private Color GetColor(Vector v)
    {
        int[] idx = GetIndexes(v);

        ushort value = Value(idx[0], idx[1], idx[2]);
        return _colorMap.GetColor(value);
    }

    private Vector GetNormal(Vector v)
    {
        int[] idx = GetIndexes(v);
        double x0 = Value(idx[0] - 1, idx[1], idx[2]);
        double x1 = Value(idx[0] + 1, idx[1], idx[2]);
        double y0 = Value(idx[0], idx[1] - 1, idx[2]);
        double y1 = Value(idx[0], idx[1] + 1, idx[2]);
        double z0 = Value(idx[0], idx[1], idx[2] - 1);
        double z1 = Value(idx[0], idx[1], idx[2] + 1);

        return new Vector(x1 - x0, y1 - y0, z1 - z0).Normalize();
    }
}