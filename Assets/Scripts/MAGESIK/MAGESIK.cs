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
        [System.Serializable]
        public class Link
        {
            public Link()
            {
            }

            public Link(float length)
            {
                this.length = length;
            }

            public Link(float length, Vector3 initialDirection)
            {
                this.length = length;
                this.initialDirection = initialDirection;
                this.previousDirection = initialDirection;
            }

            public float length = 0f;

            public Vector3 initialDirection = Vector3.up;
            
            /// <summary>
            /// The current link direction, based on world space
            /// </summary>
            public Vector3 direction = Vector3.up;
            public Vector3 previousDirection = Vector3.up;


            public Matrix4x4 RotationMatrix => Matrix4x4.Rotate(Quaternion.FromToRotation(Vector3.up, direction));
        }

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
        public List<Link> links = new List<Link>();
        [SerializeField]
        private float m_TotalLinksLength;
        [SerializeField]
        public List<IKJoint> joints = new List<IKJoint>();

        private bool m_IsInitialized = false;

        private void Awake()
        {
            Initialize();
        }

        private void Update()
        {
            if(target && target.transform.hasChanged)
            {
                SolveIK();
            }
        }

        void LateUpdate()
        {
            target.transform.hasChanged = false;
        }

        private void Initialize()
        {
            // Initialize Joints
            foreach(IKJoint joint in joints) 
            {
                if (joint == null)
                    return;

                joint.virtualPosition = joint.Position;
                joint.initialPosition = joint.virtualPosition;
                joint.virtualRotation = joint.Rotation;
                joint.initialRotation = joint.virtualRotation;

                joint.previousRotation = joint.initialRotation;
                joint.previousPosition = joint.initialPosition;
                joint.previousSolvedPosition = joint.initialPosition;
            }

            // Initialize Links
            links.Clear();
            m_TotalLinksLength = 0f;
            for(int i=1; i<joints.Count; i++)
            {
                float length = (joints[i].virtualPosition - joints[i-1].virtualPosition).magnitude;
                Vector3 direction = (joints[i].virtualPosition - joints[i-1].virtualPosition).normalized;

                links.Add(new Link(length, direction));
                m_TotalLinksLength += length;
            }

            // Store initial target data
            m_InitialTargetPosition = target.position;
            m_InitialTargetRotation = target.rotation;
            m_IsInitialized = true;

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
                SolveForward();
                SolveBackward();
            }
            PostSolve();
        }
        
        private void PreSolve()
        {
            // Store current positions as previous positions
            foreach (IKJoint j in joints)
            {
                j.previousPosition = j.virtualPosition;
                j.previousRotation = j.virtualRotation;
            }

            // Store current directions as previous directions
            foreach (Link l in links)
            {
                l.previousDirection = l.direction;
            }
        }

        private void SolveForward()
        {
            
            // Leaf to Root
            // In this step we constraint the parent joint position based on the current joint's position
            joints[joints.Count - 1].virtualPosition = TargetPosition;
            for (int i = joints.Count - 1; i > 0; i--)
            {
                // Root ... ----| previousLink |----> (parentJoint) ----| currentLink |---->  (currentJoint) ----| ... | ---->  ... Leaf
                IKJoint currentJoint = joints[i];
                IKJoint parentJoint = joints[i - 1];
                Link currentLink = links[i - 1];

                // Intermediate joint
                if (i > 1)
                {
                    Link previousLink = links[i - 2];

                    Vector3 direction = currentJoint.virtualPosition - parentJoint.virtualPosition;

                    // Constraint direction based on parent joint constraint
                    parentJoint.SolveDirectionConstraint(previousLink.RotationMatrix, direction, out direction);

                    direction = Vector3.Normalize(direction);

                    currentLink.direction = direction;
                    parentJoint.virtualPosition = currentJoint.virtualPosition - direction * currentLink.length;

                }

                // Joint connected to root
                else
                {
                    Vector3 direction = currentJoint.virtualPosition - parentJoint.virtualPosition;

                    Matrix4x4 transformMatrix = Matrix4x4.Rotate(Quaternion.FromToRotation(Vector3.up, currentLink.initialDirection));
                    parentJoint.SolveDirectionConstraint(transformMatrix, direction, out direction);

                    direction = Vector3.Normalize(direction);

                    currentLink.direction = direction;
                    parentJoint.virtualPosition = currentJoint.virtualPosition - direction * currentLink.length;

                }

            }
        }

        private void SolveBackward()
        {
            // Root to Leaf
            // In this step we constraint the child joint position based on the current joint's position

            joints[0].virtualPosition = joints[0].initialPosition; // Move root joint to initial position
            for(int i=1; i< joints.Count; i++)
            {
                IKJoint currentJoint = joints[i];
                IKJoint parentJoint = joints[i - 1];
                Link currentLink = links[i-1]; // Between parent and current

                // Parent joint is root?
                if (i == 1)
                {
                    Vector3 direction = currentJoint.virtualPosition - parentJoint.virtualPosition;

                    Matrix4x4 transformMatrix = Matrix4x4.identity;
                    parentJoint.SolveDirectionConstraint(transformMatrix, direction, out direction);
                    direction = Vector3.Normalize(direction);
                    
                    currentLink.direction = direction;
                    currentJoint.virtualPosition = parentJoint.virtualPosition + direction * currentLink.length;
                }
                else
                {
                    Link parentLink = links[i - 2];

                    Vector3 direction = currentJoint.virtualPosition - parentJoint.virtualPosition;

                    Matrix4x4 transformMatrix = parentLink.RotationMatrix;
                    parentJoint.SolveDirectionConstraint(transformMatrix, direction, out direction);
                    direction = Vector3.Normalize(direction);
                    
                    currentLink.direction = direction;
                    currentJoint.virtualPosition = parentJoint.virtualPosition + direction * currentLink.length;
                }
            }
        }


        private void PostSolve()
        {
            //Apply all virtual values to the real objects in the order from root to leaf
            for (int i = 0; i < joints.Count-1; i++)
            {
                joints[i].virtualRotation = Quaternion.FromToRotation(links[i].initialDirection, links[i].direction) * joints[i].initialRotation;
                joints[i].Rotation = joints[i].virtualRotation;
            }
            joints[joints.Count - 1].Rotation = target.rotation * Quaternion.Inverse(m_InitialTargetRotation) * joints[joints.Count - 1].initialRotation;
                
            // Apply virtual positions to the joints
            for(int i=0; i<joints.Count; i++)
            {
                joints[i].Position = joints[i].virtualPosition;
            }
        }


        private void OnDrawGizmos()
        {
            using(new Handles.DrawingScope(Color.blue, Matrix4x4.identity))
            {
                foreach(IKJoint j in joints)
                {
                    if (!j)
                        continue;
                    Handles.DrawWireCube(j.Position, Vector3.one * 0.3f * HandleUtility.GetHandleSize(j.Position));
                }
                
                for(int i=1; i<joints.Count; i++)
                {
                    if (!joints[i - 1] || !joints[i])
                        continue;
                    Handles.DrawLine(joints[i-1].Position, joints[i].Position);
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
            return (joints[joints.Count - 1].virtualPosition - target.position).sqrMagnitude < errorTolerance * errorTolerance;
        }

        /// <summary>
        /// Checks if a target is in reach of the IK joints, if the joints do not have any limit.
        /// If the joints have limits, then this function may return false positive results.
        /// </summary>
        /// <returns>Is the target in reach?</returns>
        private bool IsTargetInReach()
        {
            return Vector3.SqrMagnitude(TargetPosition - joints[0].virtualPosition) < m_TotalLinksLength * m_TotalLinksLength;
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