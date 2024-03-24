using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;

public class Planet
{
    private readonly double _radius;

    public Planet(double radius)
    {
        _radius = radius;
    }

    public double GreatCircleDistance(Vector<double> latlon1, Vector<double> latlon2)
    {
        var phi1 = latlon1[0] * Math.PI / 180.0;
        var lambda1 = latlon1[1] * Math.PI / 180.0;
        var phi2 = latlon2[0] * Math.PI / 180.0;
        var lambda2 = latlon2[1] * Math.PI / 180.0;

        var n1 = SphericalNormal(lambda1, phi1);
        var n2 = SphericalNormal(lambda2, phi2);

        var angle = Math.Acos(n1.DotProduct(n2));
        return _radius * angle;
    }

    public double GreatCircleBearing(Vector<double> latlon1, Vector<double> latlon2)
    {
        var phi1 = latlon1[0] * Math.PI / 180.0;
        var lambda1 = latlon1[1] * Math.PI / 180.0;
        var phi2 = latlon2[0] * Math.PI / 180.0;
        var lambda2 = latlon2[1] * Math.PI / 180.0;

        var n1 = SphericalNormal(lambda1, phi1);
        var n2 = SphericalNormal(lambda2, phi2);

        var east1 = Up().CrossProduct(n1);
        var north1 = n1.CrossProduct(east1);

        var nDiff2 = n2.DotProduct(n1) * n1;
        var azimVec2 = (n2 - nDiff2).Normalize();

        var cosAlpha = north1.DotProduct(azimVec2);
        var sinAlpha = east1.DotProduct(azimVec2);
        var alpha = Math.Atan2(sinAlpha, cosAlpha);
        return alpha * 180.0 / Math.PI;
    }

    // This is not a linear projection of the 3D sphere position onto the plane.
    // The dimensions are measured in meters across the surface of the sphere.
    public Vector<double> ProjOntoPlane(Vector<double> originLatlon, Vector<double> targetLatlon)
    {
        var bearing = GreatCircleBearing(originLatlon, targetLatlon) * Math.PI / 180.0;
        var distance = GreatCircleDistance(originLatlon, targetLatlon);

        var x = distance * Math.Sin(bearing); // east
        var y = distance * Math.Cos(bearing); // north
        return Vector<double>.Build.DenseOfArray(new[] { x, y });
    }

    public Vector<double> LatLonFromProj(Vector<double> originLatlon, Vector<double> proj)
    {
        var phi1 = originLatlon[0] * Math.PI / 180.0;
        var lambda1 = originLatlon[1] * Math.PI / 180.0;
        var n1 = SphericalNormal(lambda1, phi1);

        var east1 = Up().CrossProduct(n1);
        var north1 = n1.CrossProduct(east1);

        var azimVec2 = (east1.ScaleBy(proj[0]) + north1.ScaleBy(proj[1])).Normalize();

        var axis = n1.CrossProduct(azimVec2).Normalize();
        var angle = proj.L2Norm() / _radius;
        var rotationMatrix = Matrix3D.RotationAroundArbitraryVector(axis, Angle.FromRadians(angle));
        var n2 = rotationMatrix.Multiply(n1.ToVector());

        var phi2 = Math.Asin(n2[2]);
        var lambda2 = Math.Atan2(n2[1], n2[0]);
        return Vector<double>.Build.DenseOfArray(new[] { phi2 * 180.0 / Math.PI, lambda2 * 180.0 / Math.PI });
    }

    private static Vector3D SphericalNormal(double lambda, double phi)
    {
        var x = Math.Cos(phi) * Math.Cos(lambda);
        var y = Math.Cos(phi) * Math.Sin(lambda);
        var z = Math.Sin(phi);
        return new Vector3D(x, y, z);
    }

    private static Vector3D Up()
    {
        return new Vector3D(0, 0, 1);
    }
}