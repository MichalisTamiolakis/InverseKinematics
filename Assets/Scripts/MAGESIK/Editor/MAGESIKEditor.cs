using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;

namespace MAGES.IK
{
    [CustomEditor(typeof(MAGESIK))]
    public class MAGESIKEditor : Editor
    {
        protected static class Styles
        {
            public static readonly GUIContent iterations = EditorGUIUtility.TrTextContent("Solver Iterations", "In FABRIK solver how many times should the forward and backwards path be computed?");
            public static readonly GUIContent errorTolerance = EditorGUIUtility.TrTextContent("Error Tolerance", "The maximum allowable distance from the target in order to stop the remaining iterations");

            public static readonly GUIStyle singleButtonStyle = "EditModeSingleButton";
            public static readonly GUIContent editAngularLimitsButton = EditorGUIUtility.TrIconContent("d_JointAngularLimits", " | Edit Joint Angular Limits");
        }

        private static int k_ButtonHeight = 23;
        private static int k_ButtonWidth = 35;
        private static int k_SpaceBetweenLabelAndButton = 5;

        public override void OnInspectorGUI()
        {
            MAGESIK ik = (MAGESIK)target;

            bool isEditToolActive = ToolManager.activeToolType == typeof(IKConstraintsEditTool);

            if (isEditToolActive != DoEditModeInspectorModeButton("Edit Angular Limits", Styles.editAngularLimitsButton, isEditToolActive))
            {
                // Exiting Edit Mode
                if (isEditToolActive)
                {
                    ToolManager.RestorePreviousPersistentTool();
                }
                // Openning Edit Mode
                else
                {
                    ToolManager.SetActiveTool(typeof(IKConstraintsEditTool));
                }

                SceneView.RepaintAll();
            }


            EditorGUILayout.PropertyField(serializedObject.FindProperty("target"));

            EditorGUILayout.PropertyField(serializedObject.FindProperty("iterations"));

            EditorGUILayout.PropertyField(serializedObject.FindProperty("errorTolerance"));


            EditorGUILayout.PropertyField(serializedObject.FindProperty("joints"));

            serializedObject.ApplyModifiedProperties();


        }

        protected static bool DoEditModeInspectorModeButton(string label, GUIContent icon, bool value)
        {
            Rect rect = EditorGUILayout.GetControlRect(true, k_ButtonHeight, Styles.singleButtonStyle);
            Rect buttonRect = new Rect(rect.xMin + EditorGUIUtility.labelWidth, rect.yMin, k_ButtonWidth, k_ButtonHeight);

            GUIContent labelContent = new GUIContent(label);
            Vector2 labelSize = GUI.skin.label.CalcSize(labelContent);

            Rect labelRect = new Rect(
                buttonRect.xMax + k_SpaceBetweenLabelAndButton,
                rect.yMin + (rect.height - labelSize.y) * .5f,
                labelSize.x,
                rect.height);


            bool newVal = GUI.Toggle(buttonRect, value, icon, Styles.singleButtonStyle);
            GUI.Label(labelRect, label);

            return newVal;

        }



        public static void DrawPlaneAtPoint(in Plane plane, in Vector3 center, in float size)
        {
            Vector3 centerOnPlane = plane.ClosestPointOnPlane(center);
            Quaternion basis = Quaternion.LookRotation(plane.normal);
            Vector3 scale = Vector3.one * size / 10f;

            Vector3 right = Vector3.Scale(basis * Vector3.right, scale);
            Vector3 up = Vector3.Scale(basis * Vector3.up, scale);

            for (int i = -5; i <= 5; i++)
            {
                Handles.DrawLine(centerOnPlane + right * i - up * 5, centerOnPlane + right * i + up * 5);
                Handles.DrawLine(centerOnPlane + up * i - right * 5, centerOnPlane + up * i + right * 5);
            }

            Handles.DrawLine(centerOnPlane, centerOnPlane + (size / 10) * plane.normal);
        }

    }


    [EditorTool("Angular Limits Edit Tool", typeof(MAGESIK))]
    public class IKConstraintsEditTool : EditorTool
    {
        private MAGESIK m_IK;

        protected static class Styles
        {
            public static readonly GUIContent editConstraintsButton = EditorGUIUtility.IconContent("JointAngularLimits");
        }

        public override GUIContent toolbarIcon
        {
            get { return Styles.editConstraintsButton; }
        }

        protected static float GetAngularLimitHandleSize(Vector3 position)
        {
            return HandleUtility.GetHandleSize(position);
        }


        private void OnEnable()
        {
            m_IK = (MAGESIK)target;
        }

        public override void OnToolGUI(EditorWindow window)
        {
            m_IK = (MAGESIK)target;

            if (m_IK.joints.Count < 2)
                return;

            // Draw Constraints for each link and axis
            for (int i = 0; i < m_IK.joints.Count - 1; i++)
            {
                IKJoint joint = m_IK.joints[i];

                if (!joint)
                    continue;

                //if (!joint.joint || !joint.constraint)
                //    continue;

                if (i == 0)
                {
                    Matrix4x4 handleMatrix = Matrix4x4.TRS(
                                joint.Position,
                                joint.Rotation,
                                Vector3.one
                            );


                    joint.DrawHandles(handleMatrix);

                    //Matrix4x4 axisMatrix = Matrix4x4.TRS(joint.newCandidatePosition, Quaternion.identity, Vector3.one * .2f);

                    //DrawAxis(axisMatrix);
                }
                else
                {
                    //MAGESIK.Link linkToParent = m_IK.links[i - 1];

                    Vector3 linkToParentDirection = m_IK.joints[i].Position - m_IK.joints[i - 1].Position;

                    Matrix4x4 handleMatrix = Matrix4x4.TRS(
                                joint.Position,
                                Quaternion.FromToRotation(Vector3.up, linkToParentDirection),
                                Vector3.one
                            );

                    joint.DrawHandles(handleMatrix);

                    //Matrix4x4 axisMatrix = Matrix4x4.TRS(joint.newCandidatePosition, Quaternion.FromToRotation(Vector3.up, linkToParent.direction), Vector3.one * .2f);

                    //DrawAxis(axisMatrix);

                }
            }
        }

        private void DrawAxis(Matrix4x4 matrix)
        {
            using (new Handles.DrawingScope(matrix))
            {
                using (new Handles.DrawingScope(Color.green))
                {
                    Handles.DrawLine(Vector3.zero, Vector3.up);
                }
                using (new Handles.DrawingScope(Color.blue))
                {
                    Handles.DrawLine(Vector3.zero, Vector3.forward);
                }
                using (new Handles.DrawingScope(Color.red))
                {
                    Handles.DrawLine(Vector3.zero, Vector3.right);
                }
            }
        }

    }

}