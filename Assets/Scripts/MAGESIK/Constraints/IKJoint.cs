using UnityEngine;




namespace MAGES.IK
{
    [AddComponentMenu("MAGES/IK/Unconstrained Joint")]
    public class IKJoint : MonoBehaviour
    {
        public MAGESIK solver;


        // ---- These variables are set by the MAGESIK script on which thris joint isattached to ----
        // Length from this to next joint transform
        public Quaternion initialRotation;
        public Vector3 initialPosition;

        public Vector3 Position
        {
            get => transform.position;
            set => transform.position = value;
        }
        public Vector3 virtualPosition;
        public Vector3 previousPosition;
        public Vector3 previousSolvedPosition;

        public Quaternion Rotation
        {
            get => transform.rotation;
            set => transform.rotation = value;
        }
        public Quaternion virtualRotation;
        public Quaternion previousRotation;

        public IKJoint ParentJoint
        {

            get{
                if (!solver)
                    return null;

                int index = solver.Joints.IndexOf(this);
                if (index > 0)
                {
                    return solver.Joints[index - 1];
                }
                return null;
            }
        }

        /// <summary>
        /// Applies virtual position to the Transform of this joint
        /// </summary>
        public void ApplyVirtualPosition()
            {
                this.transform.position = this.virtualPosition;
            }

        /// <summary>
        /// Applies virtual rotation to the Transform of this joint
        /// </summary>
        public void ApplyVirtualRotation()
        {
            this.transform.rotation = this.virtualRotation;
        }


        /// <summary>
        /// Constraints the given direction based on the joint type.
        /// </summary>
        /// <param name="referenceTransform">The transform on which all the contraints will be referenced on</param>
        /// <param name="direction">The in unconstrained direction</param>
        /// <returns>The constrained direction based on joint type</returns>
        public virtual void SolveDirectionConstraint(in Matrix4x4 referenceTransform, in Vector3 direction, out Vector3 constrainedDirection)
        {
            constrainedDirection = direction; // No constraints for free move joint
        }

#if UNITY_EDITOR

        /// <summary>
        /// Draw Handles for this constraint.
        /// </summary>
        /// <param name="handleMatrix">The TRS matrix for the handles</param>
        /// <returns>True if changes to the constraints where made with the handles (Used to update the inspector)</returns>
        public virtual void DrawHandles(Matrix4x4 handleMatrix)
        {
        }

        /// <summary>
        /// Draws handles for this constraint based on parent direction
        /// </summary>
        public virtual void DrawHandles()
        {
            IKJoint parent = ParentJoint;

            if (!ParentJoint)
            {
                Matrix4x4 handleMatrix = Matrix4x4.Translate(
                this.initialPosition
                );


                this.DrawHandles(handleMatrix);
            }
            else
            {

                Vector3 linkToParentDirection = Position - parent.Position;

                Matrix4x4 handleMatrix = Matrix4x4.TRS(
                            Position,
                            Quaternion.FromToRotation(Vector3.up, linkToParentDirection),
                            Vector3.one
                        );

                DrawHandles(handleMatrix);
            }
        }
#endif
    }
}