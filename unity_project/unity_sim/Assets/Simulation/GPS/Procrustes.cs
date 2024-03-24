using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;

public static class Procrustes {
    // public static Matrix<double> OrthoProcrustesTransform(List<Vector<double>> fromNP, List<Vector<double>> toNP)
    // {
    //     var fromCentroid = ComputeCentroid(fromNP);
    //     var toCentroid = ComputeCentroid(toNP);

    //     var centeredFrom = CenterData(fromNP, fromCentroid);
    //     var centeredTo = CenterData(toNP, toCentroid);

    //     var covarianceMatrix = centeredFrom.Transpose() * centeredTo;

    //     var svd = covarianceMatrix.Svd(true);

    //     var rotation = svd.U * svd.VT;
    //     var scale = svd.S.Sum() / centeredFrom.PointwisePower(2).RowSums().Sum();
    //     var translation = toCentroid - (scale * rotation * fromCentroid);

    //     // Build a 3x3 matrix
    //     var transform = Matrix<double>.Build.Dense(3, 3);
    //     transform.SetSubMatrix(0, 0, rotation.Multiply(scale));
    //     transform.SetColumn(2, Vector<double>.Build.Dense(new double[] { translation[0], translation[1], 1 }));
    //     return transform;
    // }

    public static Matrix<double> MeanProcrustesTransform(List<Vector<double>> from, List<Vector<double>> to)
    {
        // This is a custom geometric implementation.
        // The canonical SVD-based implementation above did not work as expected.
        var fromCentroid = ComputeCentroid(from);
        var toCentroid = ComputeCentroid(to);

        var centeredFrom = CenterData(from, fromCentroid);
        var centeredTo = CenterData(to, toCentroid);

        var scale = 0.0;
        for (int i = 0; i < from.Count; i++)
        {
            scale += centeredTo.Row(i).L2Norm() / centeredFrom.Row(i).L2Norm();
        }
        scale /= from.Count;

        var rotationVector = Vector<double>.Build.Dense(2);
        for (int i = 0; i < from.Count; i++)
        {
            var angle = Math.Atan2(centeredTo.Row(i)[1], centeredTo.Row(i)[0]) - Math.Atan2(centeredFrom.Row(i)[1], centeredFrom.Row(i)[0]);
            rotationVector += Vector<double>.Build.Dense(new double[] { Math.Cos(angle), Math.Sin(angle) });
        }
        rotationVector /= from.Count;
        var rotation = Matrix<double>.Build.DenseOfArray(new double[,] { { rotationVector[0], -rotationVector[1] }, { rotationVector[1], rotationVector[0] } });

        var translation = toCentroid - (scale * rotation * fromCentroid);

        // Build a 3x3 matrix
        var transform = Matrix<double>.Build.Dense(3, 3);
        transform.SetSubMatrix(0, 0, rotation.Multiply(scale));
        transform.SetColumn(2, Vector<double>.Build.Dense(new double[] { translation[0], translation[1], 1 }));
        return transform;
    }

    private static Vector<double> ComputeCentroid(List<Vector<double>> vectors)
    {
        var sum = Vector<double>.Build.Dense(2);
        foreach (var vector in vectors)
        {
            sum += vector;
        }
        return sum / vectors.Count;
    }

    private static Matrix<double> CenterData(List<Vector<double>> vectors, Vector<double> centroid)
    {
        var centeredData = Matrix<double>.Build.Dense(vectors.Count, vectors[0].Count);
        for (int i = 0; i < vectors.Count; i++)
        {
            centeredData.SetRow(i, vectors[i] - centroid);
        }
        return centeredData;
    }
}