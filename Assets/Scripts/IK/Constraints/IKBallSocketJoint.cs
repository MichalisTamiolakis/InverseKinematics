using UnityEngine;

namespace IK
{
    [AddComponentMenu("IK/BallSocket Joint")]
    public class IKBallSocketJoint : IKJoint
    {
        public Vector2 minAngleDegrees = new Vector2(-90f, -90f);
        public Vector2 maxAngleDegrees = new Vector2(90f, 90f);

        public override void SolveDirectionConstraint(in Matrix4x4 referenceTransform, in Vector3 direction, out Vector3 constrainedDirection)
        {
            constrainedDirection = direction; // No constraints for free move joint
        }

        public override void DrawHandles(Matrix4x4 handleMatrix)
        {
        }


        /// <summary>
        /// Constraints the current bone's rotation based on given parent rotation
        /// </summary>
        /// <param name="parentRotation"></param>
        /// <param name="changed"></param>
        /// <returns></returns>
        public override Quaternion ConstraintRotationLocal(Quaternion rotation)
        {
            return rotation;
        }


    }
}