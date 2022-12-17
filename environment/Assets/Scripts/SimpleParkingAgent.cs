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
        transform.position = new Vector3(
            (Random.value - 0.5f) * Parameters.MAP_WIDTH,
            0.04f,
            (Random.value - 0.5f) * Parameters.MAP_HEIGHT
        );
        transform.rotation = Quaternion.Euler(0, Random.value * 360, 0);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Target and Agent positions
        sensor.AddObservation(targetTransform.localPosition.x);
        sensor.AddObservation(targetTransform.localPosition.z);
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
        if (Mathf.Sqrt(
            Mathf.Pow(transform.localPosition.x - targetTransform.localPosition.x, 2f) +
            Mathf.Pow(transform.localPosition.z - targetTransform.localPosition.z, 2f)) < 0.01
            &&
            Mathf.Abs(transform.localEulerAngles.y - 30) < 0.1)
        {
            SetReward(1f);
            EndEpisode();
        }
        else if (transform.localPosition.y < -0.1)
        {
            SetReward(-1f);
            EndEpisode();
        }
        else
        {
            AddReward(-0.0001f);
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
