using System;

namespace rt
{
    class RayTracer(Geometry[] geometries, Light[] lights)
    {
        private double ImageToViewPlane(double n, int imgSize, double viewPlaneSize)
        {
            return -n * viewPlaneSize / imgSize + viewPlaneSize / 2;
        }

        private Intersection FindFirstIntersection(Line ray, double minDist, double maxDist)
        {
            var intersection = Intersection.NONE;

            foreach (var geometry in geometries)
            {
                var intr = geometry.GetIntersection(ray, minDist, maxDist);

                if (!intr.Valid || !intr.Visible) continue;

                if (!intersection.Valid || !intersection.Visible)
                {
                    intersection = intr;
                }
                else if (intr.T < intersection.T)
                {
                    intersection = intr;
                }
            }

            return intersection;
        }

        private bool IsLit(Vector point, Light light)
        {
            const double eps = 1e-6;

            var toLight = light.Position - point;
            var distToLight = toLight.Length();
            if (distToLight <= eps) return true;

            // Nudge the start point to avoid self-intersection acne
            var start = point + toLight * (eps / distToLight);
            var shadowRay = new Line(start, light.Position);

            var occluder = FindFirstIntersection(shadowRay, eps, distToLight - eps);

            // Lit if nothing valid/visible is between the point and the light
            return !(occluder.Valid && occluder.Visible);
        }


        public void Render(Camera camera, int width, int height, string filename)
        {
            var background = new Color(0.2, 0.2, 0.2, 1.0);
            var image = new Image(width, height);

            // Camera basis
            var D = new Vector(camera.Direction).Normalize();
            var U = new Vector(camera.Up).Normalize();
            var R = (D ^ U).Normalize();               
            var viewCenter = camera.Position + D * camera.ViewPlaneDistance;

            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < height; j++)
                {
                    // Pixel - view-plane coordinates
                    double x = ImageToViewPlane(i + 0.5, width, camera.ViewPlaneWidth);
                    double y = ImageToViewPlane(j + 0.5, height, camera.ViewPlaneHeight);

                    // Point on view plane and primary ray
                    var p = viewCenter + R * x + U * y;
                    var ray = new Line(camera.Position, p);

                    var hit = FindFirstIntersection(
                        ray,
                        camera.FrontPlaneDistance,
                        camera.BackPlaneDistance
                    );

                    if (!(hit.Valid && hit.Visible))
                    {
                        image.SetPixel(i, j, background);
                        continue;
                    }

                    //  Phong shading 
                    var N = new Vector(hit.Normal).Normalize();
                    var V = (camera.Position - hit.Position).Normalize();

                    Color pixel = Color.NONE;

                    foreach (var light in lights)
                    {
                        // Ambient
                        var ambient = hit.Material.Ambient * light.Ambient;

                        // If in shadow, only ambient
                        if (!IsLit(hit.Position, light))
                        {
                            pixel += ambient;
                            continue;
                        }

                        var L = (light.Position - hit.Position).Normalize();
                        var NdotL = Math.Max(0.0, N * L);

                        // Diffuse
                        var diffuse = hit.Material.Diffuse * light.Diffuse * NdotL;

                        // Specular
                        var Rl = (N * (2.0 * (N * L)) - L).Normalize();   // reflect(L about N)
                        var RdotV = Math.Max(0.0, Rl * V);
                        var spec = hit.Material.Specular * light.Specular * Math.Pow(RdotV, hit.Material.Shininess);

                        pixel += ambient + diffuse + spec;
                    }

                    image.SetPixel(i, j, pixel);
                }
            }

            image.Store(filename);
        }
    }
}