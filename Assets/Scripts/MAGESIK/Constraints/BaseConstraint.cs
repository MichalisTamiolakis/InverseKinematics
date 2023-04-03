using UnityEngine;

namespace MAGES.IK
{
    public abstract class Constraint : MonoBehaviour
    {
        public Vector3 axis = Vector3.forward;

        public abstract Vector3 ConstraintDirectionAngle(Vector3 direction, in Matrix4x4 rotationMatrix);

#if UNITY_EDITOR
        
        /// <summary>
        /// Draw Handles for this constraint.
        /// </summary>
        /// <param name="handleMatrix">The TRS matrix for the handles</param>
        /// <returns>True if changes to the constraints where made with the handles (Used to update the inspector)</returns>
        public abstract bool DrawHandles(Matrix4x4 handleMatrix);
#endif
    }
}