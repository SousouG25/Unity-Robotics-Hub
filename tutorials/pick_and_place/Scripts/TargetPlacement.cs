using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;




namespace Unity.Robotics.PickAndPlace
{



    [RequireComponent(typeof(MeshRenderer))]
    [RequireComponent(typeof(BoxCollider))]
    public class TargetPlacement : MonoBehaviour
    {

        // <Reference Include="Assembly-CSharp">
        // <HintPath> $HOME/seatech/Unity-Robotics-Hub/tutorials/pick_and_place/PickAndPlaceProject/Library/ScriptAssemblies/Assembly-CSharp.dll>
        // </Reference>;

        List<string> k_NameExpectedTarget;

        static readonly int k_ShaderColorId = Shader.PropertyToID("_Color");
        // The threshold that the Target's speed must be under to be considered "placed" in the target area
        const float k_MaximumSpeedForStopped = 0.01f;

        /*[SerializeField]
        [Tooltip("Target object expected by this placement area. Can be left blank if only one Target in scene")]
        [SerializeField]
        [Range(0, 255)]
        [Tooltip("Alpha value for any color set during state changes.")]
        */
        int m_ColorAlpha = 100;

        MeshRenderer m_TargetMeshRenderer;

        float m_ColorAlpha01 => m_ColorAlpha / 255f;
        MeshRenderer m_MeshRenderer;
        BoxCollider m_BoxCollider;
        PlacementState m_CurrentState;
        PlacementState m_LastColoredState;

        public List<GameObject> m_Targets;


        public PlacementState CurrentState
        {
            get => m_CurrentState;
            private set
            {
                m_CurrentState = value;
                UpdateStateColor();
            }
        }

        public enum PlacementState
        {
            Outside,
            InsideFloating,
            InsidePlaced
        }

        // Start is called before the first frame update
        void Start()
        {

            //m_Targets = ListTargets.Start();
            // Check for mis-configurations and disable if something has changed without this script being updated
            // These are warnings because this script does not contain critical functionality
            // if (m_Targets == null || m_Targets.Count == 0)
            //   {
            //     m_Targets = GameObject.FindGameObjectsWithTag(k_NameExpectedTarget);
            //   }

            if (m_Targets == null || m_Targets.Count == 0)
              {
                Debug.LogWarning($"{nameof(TargetPlacement)} expects to find at least one GameObject with tag " +
                $"{k_NameExpectedTarget} to track, but did not. Can't track placement state.");
                enabled = false;
                return;
              }

              if (!TrySetComponentReferences(m_Targets.ToArray()))
              {
                enabled = false;
                return;
              }
              InitializeState();
          }

          bool TrySetComponentReferences(GameObject[] targets)
          {
              foreach (var target in targets)
              {
                  var targetMeshRenderer = target.GetComponent<MeshRenderer>();
                  if (targetMeshRenderer == null)
                  {
                      Debug.LogWarning($"{nameof(TargetPlacement)} expects a {nameof(MeshRenderer)} to be attached " +
                          $"to {k_NameExpectedTarget}. Cannot check bounds without it, so cannot track placement state.");
                      return false;
                  }
              }

              // Assume these are here because they are RequiredComponent components
              m_MeshRenderer = GetComponent<MeshRenderer>();
              m_BoxCollider = GetComponent<BoxCollider>();
              return true;
          }
          void OnValidate()
          {
              // Useful for visualizing state in editor, but doesn't wholly guarantee accurate coloring in EditMode
              // Enter PlayMode to see color update correctly
              if (m_Targets != null)
              {
                  if (TrySetComponentReferences(m_Targets.ToArray()))
                  {
                      InitializeState();
                  }
              }
          }

          void InitializeState()
          {
              bool isAnyTargetStoppedInsideBounds = false;

              foreach (var target in m_Targets)
              {
                  if (target.GetComponent<BoxCollider>().bounds.Intersects(m_BoxCollider.bounds))
                  {
                      isAnyTargetStoppedInsideBounds = true;
                      break;
                  }
              }

              if (isAnyTargetStoppedInsideBounds)
              {
                  CurrentState = IsTargetStoppedInsideBounds() ?
                      PlacementState.InsidePlaced : PlacementState.InsideFloating;
              }
              else
              {
                  CurrentState = PlacementState.Outside;
              }
          }


          void OnTriggerEnter(Collider other)
          {
              foreach (var target in m_Targets)
              {
                  if (other.gameObject.name == target.name)
                  {
                      CurrentState = PlacementState.InsideFloating;
                      break;
                  }
              }
          }

          void OnTriggerExit(Collider other)
          {
              foreach (var target in m_Targets)
              {
                  if (other.gameObject.name == target.name)
                  {
                      CurrentState = IsTargetStoppedInsideBounds() ?
                          PlacementState.InsidePlaced : PlacementState.Outside;
                      break;
                  }
              }
          }

          bool IsTargetStoppedInsideBounds()
          {
              foreach (var target in m_Targets)
              {
                  var targetIsStopped = target.GetComponent<Rigidbody>().velocity.magnitude < k_MaximumSpeedForStopped;
                  var targetIsInBounds = m_BoxCollider.bounds.Contains(target.GetComponent<MeshRenderer>().bounds.center);

                  if (targetIsStopped && targetIsInBounds)
                  {
                      return true;
                  }
              }

              return false;
          }


          // Update is called once per frame
          void Update()
          {
              bool isTargetStoppedInsideBounds = IsTargetStoppedInsideBounds();
              bool isTargetInsideBounds = m_Targets.AsEnumerable().Any(t => t.GetComponent<BoxCollider>().bounds.Intersects(m_BoxCollider.bounds));

              if (isTargetInsideBounds)
              {
                  CurrentState = isTargetStoppedInsideBounds ?
                      PlacementState.InsidePlaced : PlacementState.InsideFloating;
              }
              else
              {
                  CurrentState = PlacementState.Outside;
              }
          }



          void UpdateStateColor()
          {
              var mpb = new MaterialPropertyBlock();
              bool isAnyTargetColorUpdated = false;

              foreach (var target in m_Targets)
              {
                  var targetState = target.GetComponent<TargetPlacement>().CurrentState;
                  if (targetState != target.GetComponent<TargetPlacement>().m_LastColoredState)
                  {
                      Color stateColor;
                      switch (targetState)
                      {
                          case PlacementState.Outside:
                              stateColor = Color.red;
                              break;
                          case PlacementState.InsideFloating:
                              stateColor = Color.yellow;
                              break;
                          case PlacementState.InsidePlaced:
                              stateColor = Color.green;
                              break;
                          default:
                              Debug.LogError($"No state handling implemented for {targetState}");
                              stateColor = Color.magenta;
                              break;
                      }

                      stateColor.a = m_ColorAlpha01;
                      mpb.SetColor(k_ShaderColorId, stateColor);
                      target.GetComponent<MeshRenderer>().SetPropertyBlock(mpb);

                      target.GetComponent<TargetPlacement>().m_LastColoredState = targetState;
                      isAnyTargetColorUpdated = true;
                  }
              }

              if (!isAnyTargetColorUpdated && m_LastColoredState != CurrentState)
              {
                  // If no targets have had their color updated, update the color of this object instead
                  Color stateColor;
                  switch (CurrentState)
                  {
                      case PlacementState.Outside:
                          stateColor = Color.red;
                          break;
                      case PlacementState.InsideFloating:
                          stateColor = Color.yellow;
                          break;
                      case PlacementState.InsidePlaced:
                          stateColor = Color.green;
                          break;
                      default:
                          Debug.LogError($"No state handling implemented for {CurrentState}");
                          stateColor = Color.magenta;
                          break;
                  }

                  stateColor.a = m_ColorAlpha01;
                  mpb.SetColor(k_ShaderColorId, stateColor);
                  m_MeshRenderer.SetPropertyBlock(mpb);

                  m_LastColoredState = CurrentState;
              }
          }

}
}
