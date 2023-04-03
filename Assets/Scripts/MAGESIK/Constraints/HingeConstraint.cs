using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace MAGES.IK
{
    public class HingeConstraint : Constraint
    {
        public float minAngleDegrees = -90f;
        public float maxAngleDegrees = 90f;


        public override Vector3 ConstraintDirectionAngle(Vector3 direction, in Matrix4x4 rotationMatrix)
        {
            Vector3 projectionPlaneNormal = rotationMatrix * Vector3.left;

            // Constraint Direction on hinge plane
            direction = Vector3.ProjectOnPlane(direction, projectionPlaneNormal);

            // Constraint Direction from angles
            Vector3 startAxis = rotationMatrix * Vector3.forward;

            float angle = Vector3.SignedAngle(startAxis, direction, projectionPlaneNormal);

            // Find closest possible angle
            angle = ModularClamp(angle, minAngleDegrees, maxAngleDegrees);

            Quaternion rotation = Quaternion.AngleAxis(angle, projectionPlaneNormal);

            direction = rotation * startAxis;

            return direction;
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

#if UNITY_EDITOR

        private JointAngularLimitHandle m_AngularLimitHandle = new JointAngularLimitHandle();
        public override bool DrawHandles(Matrix4x4 handleMatrix)
        {
            m_AngularLimitHandle.xMotion = ConfigurableJointMotion.Limited;
            m_AngularLimitHandle.yMotion = ConfigurableJointMotion.Locked;
            m_AngularLimitHandle.zMotion = ConfigurableJointMotion.Locked;

            m_AngularLimitHandle.radius = 1f;//HandleUtility.GetHandleSize(joint.joint.position);

            // copy the target object's data to the handle
            m_AngularLimitHandle.xHandleColor = Color.yellow;
            m_AngularLimitHandle.xMin = minAngleDegrees;
            m_AngularLimitHandle.xMax = maxAngleDegrees;

            m_AngularLimitHandle.yHandleColor = Color.clear;
            m_AngularLimitHandle.yMin = 0f;
            m_AngularLimitHandle.yMax = 0f;

            m_AngularLimitHandle.zHandleColor = Color.clear;
            m_AngularLimitHandle.zMin = 0f;
            m_AngularLimitHandle.zMax = 0f;

            //Matrix4x4 handleMatrix = Matrix4x4.TRS(
            //    joint.newCandidatePosition,
            //    Quaternion.FromToRotation(Vector3.right, joint.axis),
            //    Vector3.one
            //);

            bool changed = false;
            using (new Handles.DrawingScope(handleMatrix))
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
                    minAngleDegrees = m_AngularLimitHandle.xMin;
                    maxAngleDegrees = m_AngularLimitHandle.xMax;

                    changed = true;
                }
            }

            return changed;
        }

#endif
    }
}
