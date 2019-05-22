using System;
using System.Collections.Generic;
using MathNet.Spatial.Euclidean;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using Q1 = MathNet.Spatial.Euclidean.Quaternion;
using Q2 = UnityEngine.Quaternion;

namespace bvh
{
    static class MathQ
    {
        public static double Deg(double rad)
        {
            return rad * 180.0 / Math.PI;
        }

        public static List<double> QuaternionToEulerYZX(double w, double x, double y, double z)
        {
            double sqw = w * w;
            double sqx = x * x;
            double sqy = y * y;
            double sqz = z * z;

            // invs (inverse square length) is only required if quaternion is not already normalised
            double invs = 1 / (sqx + sqy + sqz + sqw);
            double m00 = (sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
            double m11 = (-sqx + sqy - sqz + sqw) * invs;
            double m22 = (-sqx - sqy + sqz + sqw) * invs;

            double tmp1 = x * y;
            double tmp2 = z * w;
            double m10 = 2.0 * (tmp1 + tmp2) * invs;
            double m01 = 2.0 * (tmp1 - tmp2) * invs;

            tmp1 = x * z;
            tmp2 = y * w;
            double m20 = 2.0 * (tmp1 - tmp2) * invs;
            double m02 = 2.0 * (tmp1 + tmp2) * invs;
            tmp1 = y * z;
            tmp2 = x * w;
            double m21 = 2.0 * (tmp1 + tmp2) * invs;
            double m12 = 2.0 * (tmp1 - tmp2) * invs;

            w = Math.Sqrt(1.0 + m00 + m11 + m22) / 2.0;
            double w4 = (4.0 * w);
            x = (m21 - m12) / w4;
            y = (m02 - m20) / w4;
            z = (m10 - m01) / w4;

            if (w < 0)
            {
                w = -w; x = -x; y = -y; z = -z;
            }
            Q1 q = new Q1(w, x, z, -y);

            EulerAngles e = q.ToEulerAngles();
            //    Console.WriteLine($"q = [{w}, {x}, {y}, {z}]: rzyx = [{e.Gamma.Radians}, {e.Beta.Radians}, {e.Alpha.Radians}]");
            //   Console.WriteLine($"q = [{w}, {x}, {y}, {z}]: rzyx = [{e.Gamma.Degrees}, {e.Beta.Degrees}, {e.Alpha.Degrees}]");
            return new List<double>
            {
                e.Beta.Degrees, // y
                -e.Gamma.Degrees, // z
                e.Alpha.Degrees, // x
            };
        }

        public static List<double> QuaternionToEuler(double w, double x, double y, double z)
        {
            double sqw = w * w;
            double sqx = x * x;
            double sqy = y * y;
            double sqz = z * z;

            // invs (inverse square length) is only required if quaternion is not already normalised
            double invs = 1 / (sqx + sqy + sqz + sqw);
            double m00 = (sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
            double m11 = (-sqx + sqy - sqz + sqw) * invs;
            double m22 = (-sqx - sqy + sqz + sqw) * invs;

            double tmp1 = x * y;
            double tmp2 = z * w;
            double m10 = 2.0 * (tmp1 + tmp2) * invs;
            double m01 = 2.0 * (tmp1 - tmp2) * invs;

            tmp1 = x * z;
            tmp2 = y * w;
            double m20 = 2.0 * (tmp1 - tmp2) * invs;
            double m02 = 2.0 * (tmp1 + tmp2) * invs;
            tmp1 = y * z;
            tmp2 = x * w;
            double m21 = 2.0 * (tmp1 + tmp2) * invs;
            double m12 = 2.0 * (tmp1 - tmp2) * invs;

            w = Math.Sqrt(1.0 + m00 + m11 + m22) / 2.0;
            double w4 = (4.0 * w);
            x = (m21 - m12) / w4;
            y = (m02 - m20) / w4;
            z = (m10 - m01) / w4;

            Q1 q;
            if (w < 0) q = new Q1(-w, -x, -y, -z);
            else q = new Q1(w, x, y, z);

            EulerAngles e = q.ToEulerAngles();
            //    Console.WriteLine($"q = [{w}, {x}, {y}, {z}]: rzyx = [{e.Gamma.Radians}, {e.Beta.Radians}, {e.Alpha.Radians}]");
            //   Console.WriteLine($"q = [{w}, {x}, {y}, {z}]: rzyx = [{e.Gamma.Degrees}, {e.Beta.Degrees}, {e.Alpha.Degrees}]");
            return new List<double>
            {
                e.Gamma.Degrees, // z
                e.Beta.Degrees, // y
                e.Alpha.Degrees, // x
            };
        }

        public static Q1 EulerToQuaternion(double z, double y, double x)
        {
            double yaw = z * Math.PI / 180.0;
            double pitch = y * Math.PI / 180.0;
            double roll = x * Math.PI / 180.0;
            double cy = Math.Cos(yaw * 0.5);
            double sy = Math.Sin(yaw * 0.5);
            double cp = Math.Cos(pitch * 0.5);
            double sp = Math.Sin(pitch * 0.5);
            double cr = Math.Cos(roll * 0.5);
            double sr = Math.Sin(roll * 0.5);

            double w = cy * cp * cr + sy * sp * sr;
            double xx = cy * cp * sr - sy * sp * cr;
            double yy = sy * cp * sr + cy * sp * cr;
            double zz = sy * cp * cr - cy * sp * sr;

            return new Q1(w, xx, yy, zz);
        }

        public static List<double> YZXToZYX(double y, double z, double x)
        {
            z = z * Math.PI / 180.0;
            y = y * Math.PI / 180.0;
            x = x * Math.PI / 180.0;
            double c1 = Math.Cos(y), s1 = Math.Sin(y);
            double c2 = Math.Cos(z), s2 = Math.Sin(z);
            double c3 = Math.Cos(x), s3 = Math.Sin(x);
            double m00 = c1 * c2;
            double m01 = s1 * s3 - c1 * c3 * s2;
            double m02 = c3 * s1 + c1 * s2 * s3;
            double m10 = s2;
            double m11 = c2 * c3;
            double m12 = -c2 * s3;
            double m20 = -c2 * s1;
            double m21 = c1 * s3 + c3 * s1 * s2;
            double m22 = c1 * c3 - s1 * s2 * s3;

            double w = Math.Sqrt(1.0 + m00 + m11 + m22) / 2.0;
            double w4 = (4.0 * w);
            x = (m21 - m12) / w4;
            y = (m02 - m20) / w4;
            z = (m10 - m01) / w4;

            Q1 q;
            if (w < 0) q = new Q1(-w, -x, -y, -z);
            else q = new Q1(w, x, y, z);

            EulerAngles e = q.ToEulerAngles();
            return new List<double>
            {
                e.Gamma.Degrees, // z
                e.Beta.Degrees, // y
                e.Alpha.Degrees, // x
            };
        }

        public static double GetYRotationByZ(double w, double x, double y, double z)
        {
            Q2 q = new Q2((float)x, (float)y, (float)z, (float)w);
            Vector3 afterRotation = q * Vector3.forward;
            afterRotation.y = 0;
            afterRotation.Normalize();
            float cos = Vector3.Dot(afterRotation, Vector3.forward);
            float sin = Vector3.Cross(Vector3.forward, afterRotation).y;
            return Deg(Math.Atan2(sin, cos));
        }

        public static double GetYRotationByX(double w, double x, double y, double z)
        {
            Q2 q = new Q2((float)x, (float)y, (float)z, (float)w);
            Vector3 afterRotation = q * Vector3.right;
            afterRotation.y = 0;
            afterRotation.Normalize();
            float cos = Vector3.Dot(afterRotation, Vector3.right);
            float sin = Vector3.Cross(Vector3.right, afterRotation).y;
            return Deg(Math.Atan2(sin, cos));
        }
    }
}
