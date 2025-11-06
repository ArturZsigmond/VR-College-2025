using System;
using System.IO;
using System.Text.RegularExpressions;

namespace rt;

public class CtScan : Geometry
{
    private readonly Vector _position;
    private readonly double _scale;
    private readonly ColorMap _colorMap;
    private readonly byte[] _data;

    private readonly int[] _resolution = new int[3];
    private readonly double[] _thickness = new double[3];
    private readonly Vector _v0;
    private readonly Vector _v1;

    public CtScan(string datFile, string rawFile, Vector position, double scale, ColorMap colorMap)
        : base(
            new Material(
                ambient: new Color(0.18, 0.18, 0.18, 1.0),
                diffuse: new Color(1.20, 1.20, 1.20, 1.0),
                specular: new Color(0.20, 0.20, 0.20, 1.0),
                shininess: 12
            ),
            Color.NONE)
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
            }
            else if (kv[0] == "SliceThickness")
            {
                _thickness[0] = Convert.ToDouble(kv[1]);
                _thickness[1] = Convert.ToDouble(kv[2]);
                _thickness[2] = Convert.ToDouble(kv[3]);
            }
        }

        _v0 = position;
        _v1 = position + new Vector(
            _resolution[0] * _thickness[0] * scale,
            _resolution[1] * _thickness[1] * scale,
            _resolution[2] * _thickness[2] * scale
        );

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
        if (x < 0 || y < 0 || z < 0 ||
            x >= _resolution[0] || y >= _resolution[1] || z >= _resolution[2])
        {
            return 0;
        }

        return _data[z * _resolution[1] * _resolution[0] + y * _resolution[0] + x];
    }

    public override Intersection GetIntersection(Line line, double minDist, double maxDist)
    {
        // --- AABB for the volume ---
        var bmin = new Vector(
            Math.Min(_v0.X, _v1.X),
            Math.Min(_v0.Y, _v1.Y),
            Math.Min(_v0.Z, _v1.Z));
        var bmax = new Vector(
            Math.Max(_v0.X, _v1.X),
            Math.Max(_v0.Y, _v1.Y),
            Math.Max(_v0.Z, _v1.Z));

        var O = line.X0;
        var D = line.Dx;

        // Slab test
        double tmin = double.NegativeInfinity, tmax = double.PositiveInfinity;

        void slab(double o, double d, double minv, double maxv)
        {
            if (Math.Abs(d) < 1e-12)
            {
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

        // Clamp to interval
        const double eps = 1e-6;
        double tEnter = Math.Max(tmin, Math.Max(minDist, eps));
        double tExit = Math.Min(tmax, maxDist);
        if (tExit < tEnter) return Intersection.NONE;

        // Marching step = quarter of the smallest voxel size (world units)
        double cellX = _thickness[0] * _scale;
        double cellY = _thickness[1] * _scale;
        double cellZ = _thickness[2] * _scale;
        double baseStep = Math.Max(1e-4, Math.Min(cellX, Math.Min(cellY, cellZ)));
        double step = 0.2 * baseStep;

        // ISO threshold (ensure we actually pick up the walnut)
        byte iso = 1;

        for (double t = tEnter; t <= tExit; t += step)
        {
            var p = line.CoordinateToPosition(t);
            var idx = GetIndexes(p);

            if (idx[0] < 0 || idx[1] < 0 || idx[2] < 0 ||
                idx[0] >= _resolution[0] || idx[1] >= _resolution[1] || idx[2] >= _resolution[2])
                continue;

            var v = Value(idx[0], idx[1], idx[2]);
            if (v >= iso)
            {
                // refine surface with short binary search between last empty and first solid
                double t0 = Math.Max(t - step, tEnter);
                double a = t0, b = t;
                for (int it = 0; it < 5; it++)
                {
                    double m = 0.5 * (a + b);
                    var pm = line.CoordinateToPosition(m);
                    var im = GetIndexes(pm);

                    byte vm =
                        (im[0] < 0 || im[1] < 0 || im[2] < 0 ||
                         im[0] >= _resolution[0] || im[1] >= _resolution[1] || im[2] >= _resolution[2])
                        ? (byte)0
                        : (byte)Value(im[0], im[1], im[2]);

                    if (vm >= iso) b = m; else a = m;
                }

                double thit = 0.5 * (a + b);
                var phit = line.CoordinateToPosition(thit);
                var color = GetColor(phit);
                var normal = GetNormal(phit);

                return new Intersection(
                    valid: true,
                    visible: true,
                    geometry: this,
                    line: line,
                    t: thit,
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
        return new[]
        {
            (int)Math.Floor((v.X - _position.X) / _thickness[0] / _scale),
            (int)Math.Floor((v.Y - _position.Y) / _thickness[1] / _scale),
            (int)Math.Floor((v.Z - _position.Z) / _thickness[2] / _scale)
        };
    }

    private Color GetColor(Vector v)
    {
        int[] idx = GetIndexes(v);
        ushort value = Value(idx[0], idx[1], idx[2]);
        return _colorMap.GetColor(value); // if needed, you can force alpha=1.0 here
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

        double sx = _thickness[0] * _scale;
        double sy = _thickness[1] * _scale;
        double sz = _thickness[2] * _scale;

        var gx = (x1 - x0) / (2.0 * sx);
        var gy = (y1 - y0) / (2.0 * sy);
        var gz = (z1 - z0) / (2.0 * sz);

        return new Vector(gx, gy, gz).Normalize();
    }
}
