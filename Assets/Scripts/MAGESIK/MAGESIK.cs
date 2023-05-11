#define DEBUG

// FABRIK IK Implementation
// ------------------------
// Information about:
// - Main Paper: http://www.andreasaristidou.com/publications/papers/FABRIK.pdf
// - Constraints: http://andreasaristidou.com/publications/papers/Extending_FABRIK_with_Model_C%CE%BFnstraints.pdf

using System.Collections;
using System.Collections.Generic;
using System.Threading;
using Unity.VisualScripting;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;
using UnityEngine.UIElements;


namespace MAGES.IK
{
    [AddComponentMenu("MAGES/IK/IK Solver")]
    public class MAGESIK : MonoBehaviour
    {
        //[System.Serializable]
        //public class Link
        //{
        //    public Link()
        //    {
        //    }

        //    public Link(float length)
        //    {
        //        this.length = length;
        //    }

        //    public Link(float length, Vector3 initialDirection)
        //    {
        //        this.length = length;
        //        this.initialDirection = initialDirection;
        //        this.previousDirection = initialDirection;
        //    }

        //    public float length = 0f;

        //    public Vector3 initialDirection = Vector3.up;
            
        //    /// <summary>
        //    /// The current link direction, based on world space
        //    /// </summary>
        //    public Vector3 direction = Vector3.up;
        //    public Vector3 previousDirection = Vector3.up;


        //    public Matrix4x4 RotationMatrix => Matrix4x4.Rotate(Quaternion.FromToRotation(Vector3.up, direction));
        //}

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
            // Store current positions as previous positions
            foreach (IKJoint j in m_Joints)
            {
                j.previousPosition = j.virtualPosition;
                j.previousRotation = j.virtualRotation;
            }
        }

        private void SolveForward(Vector3 targetPos)
        {

            m_Joints[m_Joints.Count - 1].virtualPosition = targetPos;

            for(int i=m_Joints.Count- 2; i>=0; i--)
            {
                IKJoint nextJoint = m_Joints[i + 1];
                IKJoint joint = m_Joints[i];


                Vector3 direction = (nextJoint.virtualPosition - joint.virtualPosition).normalized;

                joint.virtualPosition = nextJoint.virtualPosition - direction * joint.length;
            }
        }

        private void SolveBackward(Vector3 targetPos)
        {
            m_Joints[0].virtualPosition = targetPos;
            for(int i=1; i<m_Joints.Count; i++)
            {
                IKJoint joint = m_Joints[i];
                IKJoint previousJoint = m_Joints[i - 1];

                Vector3 direction = (joint.virtualPosition - previousJoint.virtualPosition).normalized;

                joint.virtualPosition = previousJoint.virtualPosition + direction * previousJoint.length;
            
            }
        }

        private void PostSolve()
        {
            for(int i=0; i<m_Joints.Count; i++)
            {
                m_Joints[i].ApplyVirtualPosition();
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

        #endregion
        public static void DrawPlaneAtPoint(in Plane plane, in Vector3 center, in float size, in Color color, in float duration, in bool depthTest = true)
        {
            Vector3 centerOnPlane = plane.ClosestPointOnPlane(center);
            Quaternion basis = Quaternion.LookRotation(plane.normal);
            Vector3 scale = Vector3.one * size / 10f;

            Vector3 right = Vector3.Scale(basis * Vector3.right, scale);
            Vector3 up = Vector3.Scale(basis * Vector3.up, scale);

            for (int i = -5; i <= 5; i++)
            {
                UnityEngine.Debug.DrawLine(centerOnPlane + right * i - up * 5, centerOnPlane + right * i + up * 5, color, duration, depthTest);
                UnityEngine.Debug.DrawLine(centerOnPlane + up * i - right * 5, centerOnPlane + up * i + right * 5, color, duration, depthTest);
            }

            UnityEngine.Debug.DrawLine(centerOnPlane, centerOnPlane + (size / 10) * plane.normal, Color.cyan, duration, depthTest);
        }


        #region Editor

#endregion
    }
}