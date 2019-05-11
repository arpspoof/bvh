using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace bvh
{
    class Joint
    {
        public int ID;
        public string Name;
        public string Type;
        public int Parent;
        public double AttachX;
        public double AttachY;
        public double AttachZ;
        /*    public double AttachThetaX;
            public double AttachThetaY;
            public double AttachThetaZ;
            public double LimLow0;
            public double LimHigh0;
            public double LimLow1;
            public double LimHigh1;
            public double LimLow2;
            public double LimHigh2;
            public int TorqueLim;
            public int IsEndEffector;
            public int DiffWeight;*/
        public Joint ParentJoint;
        public List<Joint> ChildJoints = new List<Joint>();
    }
}
