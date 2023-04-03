using UnityEngine;

namespace MAGES.IK
{
    public class BallSocketConstraint : Constraint
    {
        public Vector2 minAngleDegrees = new Vector2(-90f, -90f);
        public Vector2 maxAngleDegrees = new Vector2(90f, 90f);

        public override Vector3 ConstraintDirectionAngle(Vector3 direction, in Matrix4x4 rotationMatrix)
        {
            throw new System.NotImplementedException();
        }

        public override bool DrawHandles(Matrix4x4 handleMatrix)
        {
            //throw new System.NotImplementedException();
            return false;
        }
    }
}