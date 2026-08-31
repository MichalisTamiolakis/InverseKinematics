using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.VisualScripting;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;

namespace IK
{
    [CustomEditor(typeof(IKJoint))]
    internal class IKJointEditor: Editor
    {
        protected static class Styles
        {
            public static readonly GUIStyle singleButtonStyle = "EditModeSingleButton";
            public static readonly GUIContent editJointButton = EditorGUIUtility.TrIconContent("d_JointAngularLimits", " | Edit Joint");
        }

        private static int k_ButtonHeight = 23;
        private static int k_ButtonWidth = 35;
        private static int k_SpaceBetweenLabelAndButton = 5;

        protected IKJoint m_Joint;

        public void OnEnable()
        {
            m_Joint= (IKJoint)target;
        }

        public override void OnInspectorGUI()
        {
           
            bool isEditToolActive = ToolManager.activeToolType == typeof(IKJointEditTool);

            if (isEditToolActive != DoEditModeInspectorModeButton("Edit Joint", Styles.editJointButton, isEditToolActive))
            {
                // Exiting Edit Mode
                if (isEditToolActive)
                {
                    ToolManager.RestorePreviousPersistentTool();
                }
                // Openning Edit Mode
                else
                {
                    ToolManager.SetActiveTool(typeof(IKJointEditTool));
                }

                SceneView.RepaintAll();
            }

            base.OnInspectorGUI();
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

    }

    [EditorTool("Edit Joint", typeof(IKJoint))]
    public class IKJointEditTool : EditorTool
    {
        private IKJoint m_Joint;

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
            m_Joint = (IKJoint)target;

        }

        public override void OnToolGUI(EditorWindow window)
        {
            m_Joint = (IKJoint)target;

            m_Joint.DrawHandles();
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
