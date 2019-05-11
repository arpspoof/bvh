﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using static bvh.MathQ;
using static bvh.Parsing;
using static bvh.BVHBasic;

namespace bvh
{
    class Program
    {
        private static List<List<double>> vExp, vAct;
        private static List<double> yRotations;

        private static string workingPath = "./data";
        private static WorkMode workMode = WorkMode.BVH;
        private static CameraOptions cameraOptions = CameraOptions.XZ_YRotSmoothed;
        private static double yRotSmoothFactor = 0.05;
        private static bool useSmoothedY = true;

        static void Main(string[] args)
        {
            string prevCommand = "";
            for (int i = 0; i < args.Length; i++)
            {
                string cmd = args[i];
                switch (prevCommand)
                {
                    case "":
                        prevCommand = InterpretCommand(cmd);
                        break;
                    case "-ySmoothFactor":
                        yRotSmoothFactor = Convert.ToDouble(cmd);
                        prevCommand = "";
                        break;
                    case "-camera":
                        // NULL, POS, POS_YRot, XZ, XZ_YRot, XZ_YRotSmoothed
                        switch (cmd)
                        {
                            case "null": cameraOptions = CameraOptions.NULL; break;
                            case "pos": cameraOptions = CameraOptions.POS; break;
                            case "pos_y": cameraOptions = CameraOptions.POS_YRot; break;
                            case "xz": cameraOptions = CameraOptions.XZ; break;
                            case "xz_y": cameraOptions = CameraOptions.XZ_YRot; break;
                            case "xz_ys": cameraOptions = CameraOptions.XZ_YRotSmoothed; break;
                        }
                        prevCommand = "";
                        break;
                    case "-p":
                        workingPath = cmd;
                        prevCommand = "";
                        break;
                }
            }
            Joint root = ParseCharacterFile($"{workingPath}/character.json");
            switch (workMode)
            {
                case WorkMode.BVH: GenBVH(root, workingPath); break;
                case WorkMode.Y: GenYFile(workingPath); break;
            }
        }

        static string InterpretCommand(string cmd)
        {
            string re = "";
            switch (cmd)
            {
                case "-bvh":
                    workMode = WorkMode.BVH;
                    break;
                case "-y":
                    workMode = WorkMode.Y;
                    break;
                case "-ySmoothFactor":
                    return cmd;
                case "-camera":
                    return cmd;
                case "-p":
                    return cmd;
                case "-noSmoothY":
                    useSmoothedY = false;
                    break;
            }
            return re;
        }

        static void GenYFile(string workingPath)
        {
            foreach (var path in Directory.EnumerateFiles($"{workingPath}/motions"))
            {
                string[] segs = path.Split('/', '\\');
                string name = segs[segs.Length - 1].Split('.')[0];

                List<List<double>> data = ParseMotionJson(path);

                List<double> yRotations = new List<double>();
                for (int i = 0; i < data.Count; i++)
                {
                    var d = data[i];
                    var qBase = QuaternionToEulerYZX(d[4], d[5], d[6], d[7]);
                    yRotations.Add(qBase[1]);
                }

                File.WriteAllText($"{workingPath}/y/{name}.txt", String.Join("\r\n", yRotations));
            }
        }

        static void GenBVH(Joint root, string workingPath)
        {
            foreach (var path in Directory.EnumerateFiles($"{workingPath}/motions"))
            {
                string[] segs = path.Split('/', '\\');
                string name = segs[segs.Length - 1].Split('.')[0];

                string yFolder = useSmoothedY ? "ys" : "y";
                yRotations = ParseYRotFile($"{workingPath}/{yFolder}/{name}.txt");

                string expPath = $"{workingPath}/v/{name}_goal.txt";
                string actPath = $"{workingPath}/v/{name}_comvel.txt";

                if (File.Exists(expPath)) vExp = ParseVectorFile(expPath);
                if (File.Exists(actPath)) vAct = ParseVectorFile(actPath);

                List<List<double>> data = ParseMotionJson(path);

                var bvhdata = ConvertMotionDataToBVHData(data);

                string bvh =
                    GenRootDescr(root) +
                    GenMotionHeader(data.Count, 1.0 / data[0][0], bvhdata[0][1]) +
                    String.Join("\n", from line in bvhdata select String.Join(" ", line));

                File.WriteAllText($"{workingPath}/bvh/{name}.bvh", bvh);
            }
        }

        static List<List<double>> ConvertMotionDataToBVHData(List<List<double>> data)
        {
            List<double> smoothedYRotations = new List<double>();
            smoothedYRotations.Add(yRotations[0]);
            double currentRot = yRotations[0];
            for (int i = 1; i < yRotations.Count; i++)
            {
                double expectedRot = yRotations[i];
                currentRot = yRotSmoothFactor * (expectedRot - currentRot) + currentRot;
                smoothedYRotations.Add(currentRot);
            }

            List<List<double>> result = new List<List<double>>();
            for (int i = 0; i < data.Count; i++)
            {
                var d = data[i];
                List<double> re = new List<double>();

                // root position(3D), 1 2 3
                // root rotation(4D), 4 5 6 7
                // chest rotation(4D), 8 9 10 11
                // neck rotation(4D), 12 13 14 15
                // right hip rotation(4D), 16 17 18 19
                // right knee rotation(1D), 20
                // right ankle rotation(4D), 21 22 23 24
                // right shoulder rotation(4D), 25 26 27 28
                // right elbow rotation(1D), 29
                // left hip rotation(4D), 30 31 32 33
                // left knee rotation(1D), 34
                // left ankle rotation(4D), 35 36 37 38
                // left shoulder rotation(4D), 39 40 41 42
                // left elbow rotation(1D), 43

                var qBase = QuaternionToEulerYZX(d[4], d[5], d[6], d[7]);
                var qRoot = QuaternionToEuler(d[4], d[5], d[6], d[7]);
                switch (cameraOptions)
                {
                    case CameraOptions.NULL:
                        re.AddRange(new List<double> { 0, 0, 0 }); // base pos
                        re.AddRange(new List<double> { 0, 0, 0 }); // base rot
                        re.AddRange(new List<double> { d[1] * 20, d[2] * 20, d[3] * 20 }); // root pos
                        re.AddRange(qRoot); // root rot*/
                        break;
                    case CameraOptions.POS:
                        re.AddRange(new List<double> { d[1] * 20, d[2] * 20, d[3] * 20 }); // base pos
                        re.AddRange(new List<double> { 0, 0, 0 }); // base rot
                        re.AddRange(new List<double> { 0, 0, 0 }); // root pos
                        re.AddRange(qRoot); // root rot*/
                        break;
                    case CameraOptions.POS_YRot:
                        re.AddRange(new List<double> { d[1] * 20, d[2] * 20, d[3] * 20 }); // base pos
                        re.AddRange(new List<double> { 0, qBase[1], 0 }); // base rot
                        re.AddRange(new List<double> { 0, 0, 0 }); // root pos
                        re.AddRange(new List<double> { qBase[0], 0, qBase[2] }); // root rot
                        break;
                    case CameraOptions.XZ:
                        re.AddRange(new List<double> { d[1] * 20, 0, d[3] * 20 }); // base pos
                        re.AddRange(new List<double> { 0, 0, 0 }); // base rot
                        re.AddRange(new List<double> { 0, d[2] * 20, 0 }); // root pos
                        re.AddRange(qRoot); // root rot
                        break;
                    case CameraOptions.XZ_YRot:
                        re.AddRange(new List<double> { d[1] * 20, 0, d[3] * 20 }); // base pos
                        re.AddRange(new List<double> { 0, qBase[1], 0 }); // base rot
                        re.AddRange(new List<double> { 0, d[2] * 20, 0 }); // root pos
                        re.AddRange(new List<double> { qBase[0], 0, qBase[2] }); // root rot
                        break;
                    case CameraOptions.XZ_YRotSmoothed:
                        re.AddRange(new List<double> { d[1] * 20, 0, d[3] * 20 }); // base pos
                        re.AddRange(new List<double> { 0, smoothedYRotations[i], 0 }); // base rot
                        re.AddRange(new List<double> { 0, d[2] * 20, 0 }); // root pos
                        double remainderYRot = qBase[1] - smoothedYRotations[i];
                        re.AddRange(YZXToZYX(remainderYRot, qBase[0], qBase[2])); // root rot
                        break;
                }

                re.AddRange(QuaternionToEuler(d[8], d[9], d[10], d[11]));
                re.AddRange(QuaternionToEuler(d[12], d[13], d[14], d[15]));
                re.AddRange(QuaternionToEuler(d[25], d[26], d[27], d[28]));
                re.AddRange(new List<double> { Deg(d[29]), 0, 0 });
                re.AddRange(new List<double> { 0, 0, 0 });
                re.AddRange(QuaternionToEuler(d[39], d[40], d[41], d[42]));
                re.AddRange(new List<double> { Deg(d[43]), 0, 0 });
                re.AddRange(new List<double> { 0, 0, 0 });
                re.AddRange(QuaternionToEuler(d[16], d[17], d[18], d[19]));
                re.AddRange(new List<double> { Deg(d[20]), 0, 0 });
                re.AddRange(QuaternionToEuler(d[21], d[22], d[23], d[24]));
                re.AddRange(QuaternionToEuler(d[30], d[31], d[32], d[33]));
                re.AddRange(new List<double> { Deg(d[34]), 0, 0 });
                re.AddRange(QuaternionToEuler(d[35], d[36], d[37], d[38]));
                
                if (vExp != null) re.AddRange(vExp[i]);
                else re.AddRange(new List<double> { 0, 0, 0 });
                if (vAct != null) re.AddRange(vAct[i]);
                else re.AddRange(new List<double> { 0, 0, 0 });

                result.Add(re);
            }
            return result;
        }
    }
}
