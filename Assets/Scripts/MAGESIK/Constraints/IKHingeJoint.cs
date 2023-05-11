using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace MAGES.IK
{
    [AddComponentMenu("MAGES/IK/Hinge Joint")]
    public class IKHingeJoint : IKJoint
    {
        public float minAngleDegrees = -90f;
        public float maxAngleDegrees = 90f;
        public Vector3 hingeAxis = Vector3.up;

        public override void SolveDirectionConstraint(in Matrix4x4 referenceTransform, in Vector3 direction, out Vector3 constrainedDirection)
        {
            Vector3 projectionPlaneNormal = referenceTransform.MultiplyVector(hingeAxis);



            // Constraint Direction on hinge plane
            constrainedDirection = Vector3.ProjectOnPlane(direction, projectionPlaneNormal);

            // Constraint Direction from angles
            Vector3 startAxis = referenceTransform * Vector3.forward;

            float angle = Vector3.SignedAngle(startAxis, constrainedDirection, projectionPlaneNormal);

            // Find closest possible angle
            angle = ModularClamp(angle, minAngleDegrees, maxAngleDegrees);

            Quaternion rotation = Quaternion.AngleAxis(angle, projectionPlaneNormal);

            constrainedDirection = rotation * startAxis;
        }

        protected static float ModularClamp(float angle, float minAngle, float maxAngle)
        {
            //Normalize angle
            angle = (angle + 180) % 360 - 180;

            //Clamp angle to min and max
            angle = Mathf.Clamp(angle, minAngle, maxAngle);

            //Return the clamped angle
            return angle;
        }

        /// <summary>
        /// Constraints the current bone's rotation based on given parent rotation
        /// </summary>
        /// <param name="rotation"></param>
        /// <param name="changed"></param>
        /// <returns></returns>
        public override Quaternion ConstraintRotationLocal(Quaternion rotation)
        {
            // If limit is zero return rotation fixed to axis
            if (minAngleDegrees == 0 && maxAngleDegrees == 0) return Quaternion.AngleAxis(0, hingeAxis);

            // Get 1 degree of freedom rotation along axis
            Quaternion free1DOF = Quaternion.FromToRotation(rotation * hingeAxis, hingeAxis) * rotation;

            return free1DOF;

        }


#if UNITY_EDITOR

        private JointAngularLimitHandle m_AngularLimitHandle = new JointAngularLimitHandle();
        
        /// <summary>
        /// Draws the handles for this joint
        /// </summary>
        /// <param name="transformMatrix">The matrix to draw the handles based on</param>
        public override void DrawHandles(Matrix4x4 transformMatrix)
        {
            m_AngularLimitHandle.xMotion = ConfigurableJointMotion.Locked;
            m_AngularLimitHandle.yMotion = ConfigurableJointMotion.Limited;
            m_AngularLimitHandle.zMotion = ConfigurableJointMotion.Locked;

            m_AngularLimitHandle.radius = 1f;//HandleUtility.GetHandleSize(joint.joint.position);

            // copy the target object's data to the handle
            m_AngularLimitHandle.yHandleColor = Color.yellow;
            m_AngularLimitHandle.yMin = minAngleDegrees;
            m_AngularLimitHandle.yMax = maxAngleDegrees;


            m_AngularLimitHandle.xHandleColor = Color.clear;
            m_AngularLimitHandle.xMin = 0f;
            m_AngularLimitHandle.xMax = 0f;


            m_AngularLimitHandle.zHandleColor = Color.clear;
            m_AngularLimitHandle.zMin = 0f;
            m_AngularLimitHandle.zMax = 0f;


            using (new Handles.DrawingScope(transformMatrix * Matrix4x4.Rotate(Quaternion.FromToRotation(Vector3.up, hingeAxis))))
            {
                // maintain a constant screen-space size for the handle's radius based on the origin of the handle matrix
                m_AngularLimitHandle.radius = HandleUtility.GetHandleSize(Vector3.zero);

                // draw the handle
                EditorGUI.BeginChangeCheck();
                m_AngularLimitHandle.DrawHandle();
                if (EditorGUI.EndChangeCheck())
                {
                    // record the target object before setting new values so changes can be undone/redone
                    Undo.RecordObject(this, "Change Constraint Properties for Hinge Joint");

                    // copy the handle's updated data back to the target object
                    minAngleDegrees = m_AngularLimitHandle.yMin;
                    maxAngleDegrees = m_AngularLimitHandle.yMax;
                }
            }
        }

#endif
    }
}
