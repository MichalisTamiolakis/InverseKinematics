using UnityEngine;
using Unity;

namespace MAGES.IK.Utilities
{
    public static class QuaternionUtilities
    {
        public static Quaternion RotationDifference(Quaternion from, Quaternion to)
        {
            if (from == to) return Quaternion.identity;

            return to * Quaternion.Inverse(from);
        }
    }

}
