#define DEBUG

// FABRIK IK Implementation
// ------------------------
// Information about:
// - Main Paper: http://www.andreasaristidou.com/publications/papers/FABRIK.pdf
// - Constraints: http://andreasaristidou.com/publications/papers/Extending_FABRIK_with_Model_C%CE%BFnstraints.pdf

using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using Unity.VisualScripting;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEngine.XR;


namespace MAGES.IK
{
    [AddComponentMenu("MAGES/IK/IK Solver")]
    public class MAGESIK : MonoBehaviour
    {
        public Transform target = null;
        public int iterations = 8;
        public float errorTolerance = .01f;


        [SerializeField]
        private Vector3 m_InitialTargetPosition;
        [SerializeField]
        private Quaternion m_InitialTargetRotation;
        private Vector3 TargetPosition => target.transform.position;
        private Quaternion TargetRotation => target.transform.rotation;


        // IK links & joints
        //public List<Link> links = new List<Link>();
        [SerializeField]
        private float m_ChainLength = 0;
        [SerializeField]
        private List<IKJoint> m_Joints = new List<IKJoint>();

        public List<IKJoint> Joints
        {
            get => m_Joints;
            set
            {
                m_Joints = value;
                // Initialize joints
                InitializeJoints();
            }
        }

        private bool m_IsInitialized = false;

        // Used to check for target transform changes in order to not update IK every frame
        Vector3 previousTargetPosition = Vector3.zero;
        Quaternion previousTargetRotation = Quaternion.identity;

        private void Awake()
        {
            Initialize();
        }

        private void Start()
        {
            if (target)
            {
                SolveIK(); // Solve once on start even if target has not moved
                HasTargetTransformChanged(); // To store initial values for previous Position and Rotation
            }
        }
        void LateUpdate()
        {
            // Check here for transform changes to detect changes
            // from animations too
            if (target && HasTargetTransformChanged())
            {
                SolveIK();
            }
        }

        public void Initialize()
        {
            InitializeJoints();

            // Store initial target data
            if (target)
            {
                m_InitialTargetPosition = target.position;
                m_InitialTargetRotation = target.rotation;
                m_IsInitialized = true;
            }

        }

        public void InitializeJoints()
        {
            m_ChainLength = 0;

            if (m_Joints.Count <= 1)
                return;

            // Initialize Joints
            int i;
            for (i= 0; i < m_Joints.Count-1; i++)
            {
                IKJoint joint = m_Joints[i];
                IKJoint nextJoint = m_Joints[i + 1];
                if (joint == null || nextJoint == null)
                    return;

                joint.virtualPosition = joint.Position;
                joint.initialPosition = joint.virtualPosition;
                joint.virtualRotation = joint.Rotation;
                joint.initialRotation = joint.virtualRotation;

                joint.previousRotation = joint.initialRotation;
                joint.previousPosition = joint.initialPosition;
                joint.previousSolvedPosition = joint.initialPosition;

                joint.solver = this;

                joint.sqrLength = Vector3.SqrMagnitude(nextJoint.Position - joint.Position);
                joint.length = Mathf.Sqrt(joint.sqrLength);
                joint.axis = Quaternion.Inverse(joint.Rotation) * (nextJoint.Position - joint.Position);

                m_ChainLength += joint.length;
            }

            m_Joints[i].virtualPosition = m_Joints[i].Position;
            m_Joints[i].initialPosition = m_Joints[i].virtualPosition;
            m_Joints[i].virtualRotation = m_Joints[i].Rotation;
            m_Joints[i].initialRotation = m_Joints[i].virtualRotation;

            m_Joints[i].previousRotation = m_Joints[i].initialRotation;
            m_Joints[i].previousPosition = m_Joints[i].initialPosition;
            m_Joints[i].previousSolvedPosition = m_Joints[i].initialPosition;
            m_Joints[i].solver = this;
            m_Joints[i].axis = Quaternion.Inverse(m_Joints[i].Rotation) * (m_Joints[i].Position - m_Joints[0].Position);

        }

        [ContextMenu("Solve IK")]
        public void SolveIK()
        {
            // Target check
            if (!m_IsInitialized || !target)
                return;

            // Reach check
            if (!IsTargetInReach())
                return;

            PreSolve();
            for(int iter=0; iter < iterations; iter++)
            {
                // Is target close enough?
                if (IsTargetWithinToleranceLimit())
                    break;

                // Solve Forward and Backward Iterations
                SolveForward(TargetPosition);
                SolveBackward(m_Joints[0].Position);
            }
            PostSolve();
        }
        
        private void PreSolve()
        {
            m_ChainLength = 0;
            for(int i= 0; i<m_Joints.Count; i++)
            {
                // Store current positions as previous positions
                m_Joints[i].previousPosition = m_Joints[i].virtualPosition;
                m_Joints[i].previousRotation = m_Joints[i].virtualRotation;

                m_Joints[i].virtualPosition = m_Joints[i].Position;
                m_Joints[i].virtualRotation = m_Joints[i].Rotation;

                // Recalculate and store lengths for any change between frames
                if (i < m_Joints.Count - 1)
                {
                    m_Joints[i].length = (m_Joints[i].Position - m_Joints[i + 1].Position).magnitude;
                    m_Joints[i].axis = Quaternion.Inverse(m_Joints[i].Rotation) * (m_Joints[i + 1].Position - m_Joints[i].Position);

                    m_ChainLength += m_Joints[i].length;
                }
                
                m_Joints[i].localPosition = Quaternion.Inverse(GetParentVirtualRotation(i)) * (m_Joints[i].transform.position - GetParentVirtualPosition(i));
            }
        }

        private void SolveForward(Vector3 targetPos)
        {

            m_Joints[m_Joints.Count - 1].virtualPosition = targetPos;

            for(int i=m_Joints.Count- 2; i>=0; i--)
            {
                IKJoint nextJoint = m_Joints[i + 1];
                IKJoint joint = m_Joints[i];

                joint.virtualPosition = nextJoint.virtualPosition - (nextJoint.virtualPosition - joint.virtualPosition).normalized * joint.length;


                ConstraintJointRotationForward(i, i + 1);
            }
            
            ConstraintJointRotationForward(0, 0);
        }

        private void SolveBackward(Vector3 targetPos)
        {
            m_Joints[0].virtualPosition = targetPos;

            for (int i = 0; i < m_Joints.Count - 1; i++)
            {
                Vector3 nextPosition = m_Joints[i].virtualPosition + (m_Joints[i + 1].virtualPosition - m_Joints[i].virtualPosition).normalized * m_Joints[i].length; 

                Quaternion swing = Quaternion.FromToRotation(m_Joints[i].virtualRotation * m_Joints[i].axis, nextPosition - m_Joints[i].virtualPosition);
                Quaternion unconstrainedRotation = swing * m_Joints[i].virtualRotation;

                Quaternion constrainedLocalRotation = GetJointConstrainedRotation(i, unconstrainedRotation, out _);
                //Quaternion constrainedLocalRotation = unconstrainedRotation;


                Quaternion fromTo = Utilities.QuaternionUtilities.RotationDifference(m_Joints[i].virtualRotation, constrainedLocalRotation);
                m_Joints[i].virtualRotation = constrainedLocalRotation;
                RotateChildrenJoints(i, fromTo);

                m_Joints[i + 1].virtualPosition = m_Joints[i].virtualPosition + m_Joints[i].virtualRotation * m_Joints[i + 1].localPosition;
            }

            // Reconstruct solver rotations to protect from invalid Quaternions
            for (int i = 0; i < m_Joints.Count; i++)
            {
                m_Joints[i].virtualRotation = Quaternion.LookRotation(m_Joints[i].virtualRotation * Vector3.forward, m_Joints[i].virtualRotation * Vector3.up);
            }

        }


        private void PostSolve()
        {
            m_Joints[0].transform.position = m_Joints[0].virtualPosition;

            for (int i = 0; i < m_Joints.Count-1; i++)
            {
                m_Joints[i].transform.rotation = m_Joints[i].virtualRotation;
            }
        }

        private void OnDrawGizmos()
        {
            using(new Handles.DrawingScope(Color.blue, Matrix4x4.identity))
            {
                foreach(IKJoint j in m_Joints)
                {
                    if (!j)
                        continue;
                    Handles.DrawWireCube(j.Position, Vector3.one * 0.3f * HandleUtility.GetHandleSize(j.Position));
                }
                
                for(int i=1; i<m_Joints.Count; i++)
                {
                    if (!m_Joints[i - 1] || !m_Joints[i])
                        continue;
                    Handles.DrawLine(m_Joints[i-1].Position, m_Joints[i].Position);
                }
            }

        }

        #region Helper Functions


        /// <summary>
        /// Checks if the target is closer than the tolerance distance
        /// </summary>
        /// <returns>True if target is closer than tolerance distance, false otherwise</returns>
        private bool IsTargetWithinToleranceLimit()
        {
            return (m_Joints[m_Joints.Count - 1].virtualPosition - target.position).sqrMagnitude < errorTolerance * errorTolerance;
        }

        /// <summary>
        /// Checks if a target is in reach of the IK joints, if the joints do not have any limit.
        /// If the joints have limits, then this function may return false positive results.
        /// </summary>
        /// <returns>Is the target in reach?</returns>
        private bool IsTargetInReach()
        {
            return Vector3.SqrMagnitude(TargetPosition - m_Joints[0].virtualPosition) < m_ChainLength * m_ChainLength;
        }

        /// <summary>
        /// Detects if target transform (only position or rotation) has changed
        /// </summary>
        /// <returns>True if the transform has changed since the last time this function was called, false otherwise</returns>
        private bool HasTargetTransformChanged()
        {
            bool changed =  previousTargetPosition != TargetPosition || previousTargetRotation != TargetRotation;
            previousTargetPosition = TargetPosition;
            previousTargetRotation = TargetRotation;
            return changed;
        }

        /// <summary>
        /// Constraint joint direction, used in Forward Reach section
        /// </summary>
        /// <param name="baseTransformJointIndex"></param>
        /// <param name="jointIndex"></param>
        private void ConstraintJointRotationForward(int baseTransformJointIndex, int jointIndex)
        {
            // Store last bone's position before limiting the rotation
            Vector3 lastBoneBeforeRotationLimit = m_Joints[m_Joints.Count - 1].virtualPosition;

            // Rotate all bones to their new rotation
            for (int i = baseTransformJointIndex; i < m_Joints.Count - 1; i++)
            {
                Quaternion fromTo = Quaternion.FromToRotation(m_Joints[i].virtualRotation * m_Joints[i].axis, m_Joints[i + 1].virtualPosition - m_Joints[i].virtualPosition);

                m_Joints[i].virtualRotation = fromTo * m_Joints[i].virtualRotation;
            }

            // Limit bone's rotation
            bool changed;
            Quaternion afterLimit = GetJointConstrainedRotation(jointIndex, m_Joints[jointIndex].virtualRotation, out changed);

            if (changed)
            {
                // Rotating and positioning the hierarchy so that the last bone's position is maintained
                if (jointIndex < m_Joints.Count - 1)
                {
                    Quaternion change = Utilities.QuaternionUtilities.RotationDifference(m_Joints[jointIndex].virtualRotation, afterLimit);
                    m_Joints[jointIndex].virtualRotation = afterLimit;


                    // Rotate all links around the joint 
                    RotateChildrenJoints(jointIndex, change); // First rotate directions
                    MoveChildrenAroundJoint(jointIndex, change); // Move positions 

                    // Rotating to compensate for the limit
                    Quaternion fromTo = Quaternion.FromToRotation(m_Joints[m_Joints.Count - 1].virtualPosition - m_Joints[baseTransformJointIndex].virtualPosition, lastBoneBeforeRotationLimit - m_Joints[baseTransformJointIndex].virtualPosition);

                    RotateJoint(baseTransformJointIndex, fromTo);
                    RotateChildrenJoints(baseTransformJointIndex, fromTo);
                    MoveChildrenAroundJoint(baseTransformJointIndex, fromTo);

                    // Moving the bone so that last bone maintains it's initial position
                    MoveJoint(baseTransformJointIndex, lastBoneBeforeRotationLimit - m_Joints[m_Joints.Count - 1].virtualPosition);
                    MoveChildrenJoints(baseTransformJointIndex, lastBoneBeforeRotationLimit - m_Joints[m_Joints.Count - 1].virtualPosition);

                }
                else
                {
                    // last bone
                    m_Joints[jointIndex].virtualRotation = afterLimit;
                }
            }
        }

        /// <summary>
        /// Get the constrained rotation of a joint (based on joint type), given an unconstrained one
        /// </summary>
        /// <param name="jointIndex"></param>
        /// <param name="unconstrainedRotation"></param>
        /// <param name="changed"></param>
        /// <returns></returns>
        private Quaternion GetJointConstrainedRotation(int jointIndex, Quaternion unconstrainedRotation, out bool changed)
        {
            changed = false;

            Quaternion parentRotation = GetParentVirtualRotation(jointIndex);

            // Convert actual rotation to local rotation space
            Quaternion localSpaceUnconstrainedRotation = Quaternion.Inverse(parentRotation) * unconstrainedRotation;

            Quaternion localSpaceConstrainedRotation = m_Joints[jointIndex].ConstraintRotationLocal(localSpaceUnconstrainedRotation, out changed);

            return parentRotation * localSpaceConstrainedRotation;

        }

        /// <summary>
        /// Get the parent joint solver rotation
        /// </summary>
        /// <param name="jointIndex"></param>
        /// <returns></returns>
        private Quaternion GetParentVirtualRotation(int jointIndex)
        {
            if (jointIndex > 0) return m_Joints[jointIndex - 1].virtualRotation;
            if (m_Joints[0].transform.parent == null) return Quaternion.identity;
            return m_Joints[0].transform.parent.rotation;
        }

        /// <summary>
        /// Get the parent joint solver position
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        private Vector3 GetParentVirtualPosition(int index)
        {
            if (index > 0) return m_Joints[index - 1].virtualPosition;
            if (m_Joints[0].transform.parent == null) return Vector3.zero;
            return m_Joints[0].transform.parent.position;
        }

        /// <summary>
        /// Rotate given joint by rotation
        /// </summary>
        /// <param name="jointIndex"></param>
        /// <param name="rotation"></param>
        private void RotateJoint(int jointIndex, Quaternion rotation)
        {
            m_Joints[jointIndex].virtualRotation = rotation * m_Joints[jointIndex].virtualRotation;
        }

        /// <summary>
        /// Rotates children joints by given rotation
        /// </summary>
        /// <param name="jointIndex"></param>
        /// <param name="rotation"></param>
        private void RotateChildrenJoints(int jointIndex, Quaternion rotation)
        {
            for(int i=jointIndex +1; i<m_Joints.Count; i++)
            {
                m_Joints[i].virtualRotation = rotation * m_Joints[i].virtualRotation;
            }
        }

        /// <summary>
        /// Moves children positions around a joint, by given rotation
        /// </summary>
        /// <param name="jointIndex"></param>
        /// <param name="rotation"></param>
        private void MoveChildrenAroundJoint(int jointIndex, Quaternion rotation)
        {
            for (int i = jointIndex + 1; i < m_Joints.Count; i++)
            {
                Vector3 dir = m_Joints[i].virtualPosition - m_Joints[jointIndex].virtualPosition;
                m_Joints[i].virtualPosition = m_Joints[jointIndex].virtualPosition + rotation * dir;
            }
        }

        /// <summary>
        /// Moves joint by given offset
        /// </summary>
        /// <param name="jointIndex"></param>
        /// <param name="offset"></param>
        private void MoveJoint(int jointIndex, Vector3 offset)
        {
            m_Joints[jointIndex].virtualPosition += offset;
        }

        /// <summary>
        /// Moves children joint positions by given offset
        /// </summary>
        /// <param name="jointIndex"></param>
        /// <param name="offset"></param>
        private void MoveChildrenJoints(int jointIndex, Vector3 offset)
        {
            for(int i = jointIndex+1; i<m_Joints.Count; i++)
            {
                MoveJoint(i, offset);
            }
        }
        #endregion

        #region Editor

        #endregion
    }
}