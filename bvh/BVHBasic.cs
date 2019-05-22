
namespace bvh
{
    static class BVHBasic
    {
        public static string GenRootDescr(Joint r)
        {
            string inner = "";
            foreach (Joint c in r.ChildJoints)
            {
                inner += GenJointDescr(c, 1);
            }
            return $"HIERARCHY\n" +
                $"ROOT {r.Name}\n" +
                $"{{\n" +
                $"\tOFFSET 0.0 0.0 0.0\n" +
                $"\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation\n" +
                $"{inner}" +
                $"}}\n";
        }

        public static string GenJointDescr(Joint j, int level)
        {
            string tabs = "";
            for (int i = 0; i < level; i++) tabs += "\t";
            string tabs1 = tabs + "\t";

            if (j == null)
            {
                return $"{tabs}End Site\n" +
                    $"{tabs}{{\n" +
                    $"{tabs1}OFFSET 0.0 0.0 0.0\n" +
                    $"{tabs}}}\n";
            }

            string inner = "";
            foreach (Joint c in j.ChildJoints)
            {
                inner += GenJointDescr(c, level + 1);
            }
            if (j.ChildJoints.Count == 0)
            {
                inner = GenJointDescr(null, level + 1);
            }

            string channel = $"{tabs1}CHANNELS 3 Zrotation Yrotation Xrotation\n";
            if (j.Name.ToLower() == "root") channel = $"{tabs1}CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation\n";

            return $"{tabs}JOINT {j.Name}\n" +
                $"{tabs}{{\n" +
                $"{tabs1}OFFSET {j.AttachX * 20} {j.AttachY * 20} {j.AttachZ * 20}\n" +
                channel
                +
                $"{inner}" +
                $"{tabs}}}\n";
        }

        public static string GenMotionHeader(int nFrame, double fps)
        {
            return $"MOTION\n" +
                $"Frames: {nFrame + 1}\n" +
                $"Frame Time: {1.0 / fps}\n" +
                $"0 18.08466 0 0 0 0 0 0 0 0 -90 0 0 0 0 0 0 0 0 0 -90 0 0 0 0 0 0 0 0 90 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\n";
        }
    }
}
