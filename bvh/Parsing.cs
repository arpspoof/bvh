using MathNet.Spatial.Euclidean;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.IO;

namespace bvh
{
    static class Parsing
    {
        public static Joint ParseCharacterFile(string path)
        {
            string characterJson = File.ReadAllText(path);
            dynamic characterObj = JsonConvert.DeserializeObject(characterJson);
            dynamic jointsObj = characterObj["Skeleton"]["Joints"];
            List<Joint> jointsDef = new List<Joint>();
            foreach (dynamic jointObj in jointsObj)
            {
                jointsDef.Add(new Joint
                {
                    ID = jointObj["ID"],
                    Name = jointObj["Name"],
                    Type = jointObj["Type"],
                    Parent = jointObj["Parent"],
                    AttachX = jointObj["AttachX"],
                    AttachY = jointObj["AttachY"],
                    AttachZ = jointObj["AttachZ"],
                });
            }
            Joint root = jointsDef[0];
            for (int i = 1; i < jointsDef.Count; i++)
            {
                var currentJoint = jointsDef[i];
                var parent = jointsDef[currentJoint.Parent];
                parent.ChildJoints.Add(currentJoint);
                currentJoint.ParentJoint = parent;
            }
            return root;
        }

        public static List<List<double>> ParseMotionJson(string path)
        {
            string json = File.ReadAllText(path);
            dynamic obj = JsonConvert.DeserializeObject<dynamic>(json);
            dynamic data = obj["Frames"];
            List<List<double>> result = new List<List<double>>();
            foreach (dynamic innerList in data)
            {
                List<double> re = new List<double>();
                foreach (double d in innerList)
                {
                    re.Add(d);
                }
                result.Add(re);
            }
            return result;
        }

        public static List<List<double>> ParseVectorFile(string path)
        {
            List<List<double>> res = new List<List<double>>();
            StreamReader r = new StreamReader(path);
            while (!r.EndOfStream)
            {
                string line = r.ReadLine().Trim();
                string[] segs = line.Split(' ');
                if (segs.Length != 3) throw new Exception("spaces");
                List<double> d = new List<double>();
                d.Add(Convert.ToDouble(segs[0]));
                d.Add(0);
                d.Add(Convert.ToDouble(segs[2]));
                Vector3D v = new Vector3D(d);
                Vector3D u = v.Normalize().ToVector3D();
                double cos = new Vector3D(1, 0, 0).DotProduct(u);
                double sin = new Vector3D(1, 0, 0).CrossProduct(u).Y;
                double rotY = Math.Atan2(sin, cos) * 180.0 / Math.PI;
                res.Add(new List<double> { 0, rotY, 0 });
            }
            r.Close();
            return res;
        }

        public static List<double> ParseYRotFile(string path)
        {
            var re = new List<double>();
            StreamReader r = new StreamReader(path);
            while (!r.EndOfStream)
            {
                string line = r.ReadLine().Trim();
                if (line == "") continue;
                double d = Convert.ToDouble(line);
                re.Add(d);
            }
            return re;
        }
    }
}
