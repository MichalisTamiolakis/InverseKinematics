using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;

namespace IK
{
    [CustomEditor(typeof(IKController))]
    public class IKControllerEditor : Editor
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
            IKController ik = (IKController)target;

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


            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("m_Joints"));
            if (EditorGUI.EndChangeCheck())
            {
                ik.InitializeJoints();
            }

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


    [EditorTool("Angular Limits Edit Tool", typeof(IKController))]
    public class IKConstraintsEditTool : EditorTool
    {
        private IKController m_IK;

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
            m_IK = (IKController)target;
        }

        public override void OnToolGUI(EditorWindow window)
        {
            m_IK = (IKController)target;

            if (m_IK.Joints.Count < 2)
                return;

            // Draw Constraints for each link and axis
            foreach(IKJoint joint in m_IK.Joints)
            {
                joint.DrawHandles();
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