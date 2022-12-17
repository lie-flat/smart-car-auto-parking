using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class SimpleParkingAgent : Agent
{
    private SimpleCarController carController;
    private Rigidbody rBody;

    public GameObject Target;
    private MeshCollider targetMesh;
    private Transform targetTransform;
    public float targetX = -0.068f;
    public float targetZ = -0.056f;
    public float targetR = -30f;

    [SerializeField] private float distance = float.PositiveInfinity;
    // Start is called before the first frame update
    void Start()
    {
        carController = GetComponent<SimpleCarController>();
        rBody = GetComponent<Rigidbody>();
        targetMesh = Target.GetComponent<MeshCollider>();
        targetTransform = Target.transform;
    }

    public override void OnEpisodeBegin()
    {
        // Random spawn
        transform.localPosition = new Vector3(
            // Spawn on 80% center area of the map
            0.8f * (Random.value - 0.5f) * Parameters.MAP_WIDTH,
            0.04f,
            0.8f * (Random.value - 0.5f) * Parameters.MAP_HEIGHT
        );
        transform.rotation = Quaternion.Euler(0, Random.value * 360, 0);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Target and Agent positions
        sensor.AddObservation(targetX);
        sensor.AddObservation(targetZ);
        sensor.AddObservation(transform.localPosition.x);
        sensor.AddObservation(transform.localPosition.z);

        // Agent rotation
        sensor.AddObservation(transform.localEulerAngles.y / (2 * Mathf.PI));

        // Agent velocity
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var values = actions.ContinuousActions;
        var steering = values[0];
        var motor = values[1];
        // Debug.Log($"{steering}, {motor}");
        carController.Control(steering, motor);
        distance = Mathf.Sqrt(
            Mathf.Pow(transform.localPosition.x - targetX, 2f) +
            Mathf.Pow(transform.localPosition.z - targetZ, 2f));
        if (distance < 0.03
            // && Mathf.Abs(transform.localEulerAngles.y - 30) < 0.1
            )
        {
            SetReward(1f);
            EndEpisode();
        }
        else if (distance > 1)
        {
            // Too far
            AddReward(-0.001f * distance);
        }
        else if (transform.localPosition.y < 0)
        {
            // Fall
            SetReward(-1f);
            EndEpisode();
        }
        else
        {
            AddReward(-0.00001f);
        }
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

        float steering = Input.GetAxis("Horizontal"); //-1 to 1
        float motor = Input.GetAxis("Vertical");  //0 to 1
        // Debug.Log($"{accel}, {steering}");
        // Input from network is between -1 to 1, map values accordingly
        // accel = accel * 2 - 1;
        continuousActionsOut[0] = steering;
        continuousActionsOut[1] = motor;
    }

}
