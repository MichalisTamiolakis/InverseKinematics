using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace MAGES.IK
{
    [ExecuteInEditMode]
    public class MAGESIK : MonoBehaviour
    {
        [System.Serializable]
        public class Joint
        {
            public Transform joint;

            public Vector3 initialPosition;
            public Quaternion initialRotation;

            public Vector3 newCandidatePosition;
            public Quaternion newCandidateRotation;

            [HideInInspector]
            public Constraint constraint;
        }

        [System.Serializable]
        public class Link
        {
            public float length;

            public Vector3 initialDirection;
            /// <summary>
            /// The current link direction
            /// </summary>
            public Vector3 direction;


            public Vector3 newCandidateDirection;
        }

        public Transform target = null;
        public List<Joint> joints = new List<Joint>();
        public int iterations = 10;
        public float errorTolerance = .01f;

        [SerializeField]
        private float m_TotalLength;
        [SerializeField]
        public Link[] links { private set; get;}

        [SerializeField]
        private Vector3 m_InitialTargetPosition;
        [SerializeField]
        private Quaternion m_InitialTargetRotation;

        public bool debug = true;


        private void Awake()
        {
        }


        // Start is called before the first frame update
        void Start()
        {
            if (Application.isPlaying)
            {
                Initialize();

                SolveIK();
            }
        }

        private void Update()
        {
            if (!Application.isPlaying)
            {
                Initialize();
            }
        }

        // Update is called once per frame
        void LateUpdate()
        {
            if (Application.isPlaying && target.transform.hasChanged)
            {
                SolveIK();
                target.transform.hasChanged = false;
            }
            //if ( && Input.GetKeyDown(KeyCode.U))
            //{
            //}
        }

        private void Initialize()
        {
            if (!target)
                return;

            // Store current position as previous position
            foreach (Joint j in joints)
            {
                if (!j.joint)
                    return;

                // Get Constraints if available
                if (j.joint.gameObject.TryGetComponent(out Constraint constraint))
                {
                    j.constraint = constraint;
                }
                else
                {
                    j.constraint = null;
                }

                j.initialPosition = j.joint.position;
                j.initialRotation = j.joint.rotation;

                j.newCandidatePosition = j.joint.position;
                j.newCandidateRotation = j.joint.rotation;
            }


            links = new Link[joints.Count - 1];
            m_TotalLength = 0;
            for (int i = 0; i < joints.Count - 1; i++)
            {
                float currLength = Vector3.Distance(joints[i + 1].joint.position, joints[i].joint.position);
                links[i] = new Link();
                links[i].length = currLength;
                links[i].direction = links[i].initialDirection = joints[i + 1].joint.position - joints[i].joint.position;
                m_TotalLength += currLength;
            }

            // Store initial target data
            m_InitialTargetPosition = target.position;
            m_InitialTargetRotation = target.rotation;
        }

        public void SolveIK()
        {

            // Store current position as previous position
            foreach (Joint j in joints)
            {
                j.newCandidatePosition = j.joint.position;
                j.newCandidateRotation = j.joint.rotation;
            }

            float targetDistanceSquared = Vector3.SqrMagnitude(target.position - joints[0].newCandidatePosition);

            for (int iter = 0; iter < iterations; iter++)
            {
                #region forward iteration


                // Leaf to Root
                // In this step we constraint the parent joint position based on the current joint's position
                joints[joints.Count - 1].newCandidatePosition = target.transform.position;
                for (int i = joints.Count - 1; i > 0; i--)
                {
                    // (...) ----| previousLink |----> (parrentJoint) ----| currentLink |---->  (currentJoint) ----| ... | ---->  
                    Joint currentJoint = joints[i];
                    Joint parentJoint = joints[i - 1];
                    Link currentLink = links[i - 1];

                    // Intermediate joint
                    if (i > 1)
                    {
                        Link previousLink = links[i - 2];

                        Vector3 direction = currentJoint.newCandidatePosition - parentJoint.newCandidatePosition;

                        // Constraint direction based on hinge type
                        if (parentJoint.constraint)
                        {
                            Quaternion hingeRotation = Quaternion.FromToRotation(Vector3.up, previousLink.newCandidateDirection) * Quaternion.FromToRotation(Vector3.right, parentJoint.constraint.axis);
                            direction = parentJoint.constraint.ConstraintDirectionAngle(direction, Matrix4x4.Rotate(hingeRotation));
                        }

                        direction = Vector3.Normalize(direction);
                        currentLink.newCandidateDirection = direction;


                        // Place parent joint the constrained distance away in the direction
                        parentJoint.newCandidatePosition = currentJoint.newCandidatePosition - direction * currentLink.length;

                    }

                    // Joint connected to root
                    else
                    {
                        Vector3 direction = currentJoint.newCandidatePosition - parentJoint.newCandidatePosition;


                        // Constraint direction based on hinge type
                        if (parentJoint.constraint)
                        {
                            Quaternion hingeRotation = Quaternion.FromToRotation(Vector3.up, currentLink.initialDirection) * Quaternion.FromToRotation(Vector3.right, parentJoint.constraint.axis);
                            direction = parentJoint.constraint.ConstraintDirectionAngle(direction, Matrix4x4.Rotate(hingeRotation));
                        }

                        direction = Vector3.Normalize(direction);

                        currentLink.newCandidateDirection = direction;


                        // Place parent joint the constrained distance away in the direction
                        parentJoint.newCandidatePosition = currentJoint.newCandidatePosition - direction * currentLink.length;

                    }

                }
                #endregion

                #region backwards iteration
                // Root to Leaf
                // In this step we constraint the child joint position based on the current joint's position

                // Move root node to the initial position
                joints[0].newCandidatePosition = joints[0].initialPosition;

                for (int i = 0; i < joints.Count - 1; i++)
                {

                    // (...) ----| previousLink |----> (currentJoint) ----| currentLink |---->  (childJoint) ----| ... | ---->             
                    Joint currentJoint = joints[i];
                    Joint childJoint = joints[i + 1];
                    Link currentLink = links[i];

                    // Root joint
                    if (i == 0)
                    {
                        Vector3 direction = childJoint.newCandidatePosition - currentJoint.newCandidatePosition;

                        // Constraint direction based on hinge type
                        if (currentJoint.constraint)
                        {
                            Quaternion constraintRotation = Quaternion.FromToRotation(Vector3.right, currentJoint.constraint.axis);
                            direction = currentJoint.constraint.ConstraintDirectionAngle(direction, Matrix4x4.Rotate(constraintRotation));
                        }

                        direction = Vector3.Normalize(direction);
                        currentLink.newCandidateDirection = direction;

                        // Place child joint the constrained distance away in the direction
                        childJoint.newCandidatePosition = currentJoint.newCandidatePosition + direction * currentLink.length;

                    }

                    // Intermediate joints
                    else
                    {
                        Link previousLink = links[i - 1];

                        Vector3 direction = childJoint.newCandidatePosition - currentJoint.newCandidatePosition;

                        // Constraint direction based on hinge type
                        if (currentJoint.constraint)
                        {
                            Quaternion hingeRotation = Quaternion.FromToRotation(Vector3.up, previousLink.newCandidateDirection) * Quaternion.FromToRotation(Vector3.right, currentJoint.constraint.axis);
                            direction = currentJoint.constraint.ConstraintDirectionAngle(direction, Matrix4x4.Rotate(hingeRotation));
                        }

                        direction = Vector3.Normalize(direction);
                        currentLink.newCandidateDirection = direction;

                        //Debug.DrawLine(currentJoint.newCandidatePosition, currentJoint.newCandidatePosition + direction * 5f, Color.red, 1f);
                        // Place parent joint the constrained distance away in the direction
                        childJoint.newCandidatePosition = currentJoint.newCandidatePosition + direction * currentLink.length;
                    }

                }
                #endregion

                // The target is close enough do not do other iterations
                if ((joints[joints.Count - 1].newCandidatePosition - target.position).sqrMagnitude < errorTolerance * errorTolerance)
                {
                    break;
                }
            }


            // Is the new solution actually better than the old one?
            if ((target.position - joints[joints.Count-1].newCandidatePosition).sqrMagnitude < (target.position - joints[joints.Count-1].joint.position).sqrMagnitude) {
                // Calculate Rotations
                for (int i = 0; i < joints.Count - 1; i++)
                {
                    joints[i].newCandidateRotation = Quaternion.FromToRotation(links[i].initialDirection, joints[i + 1].newCandidatePosition - joints[i].newCandidatePosition) * joints[i].initialRotation;
                }
                joints[joints.Count - 1].newCandidateRotation = target.rotation * Quaternion.Inverse(m_InitialTargetRotation) * joints[joints.Count - 1].initialRotation;


                // Apply new candidate positions and rotations back to transforms
                foreach (Joint j in joints)
                {
                    j.joint.position = j.newCandidatePosition;
                    j.joint.rotation = j.newCandidateRotation;
                }

                for (int i = 0; i < joints.Count - 1; i++)
                {
                    links[i].direction = joints[i + 1].joint.position - joints[i].joint.position;
                }
            }
            else
            {
                // Apply previous positions & Rotations
                foreach (Joint j in joints)
                {
                    j.newCandidatePosition = j.joint.position;
                    j.newCandidateRotation = j.joint.rotation;
                }

                for (int i = 0; i < joints.Count - 1; i++)
                {
                    links[i].newCandidateDirection = links[i].direction;
                }
            }
        }


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