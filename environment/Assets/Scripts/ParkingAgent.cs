using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

public class ParkingAgent : Agent
{
    private CarController carController;
    private Rigidbody rBody;

    public GameObject Target;
    public float motorMultipler = 0.1f;
    private MeshCollider targetMesh;
    private Transform targetTransform;

    // Start is called before the first frame update
    void Start()
    {
        carController = GetComponent<CarController>();
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
        var motor = values[0];
        var turn = values[1];
        carController.Move(turn * Parameters.ANGLE_LIM, motor * motorMultipler, 0, 0);
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
        float accel = Input.GetAxis("Vertical");  //0 to 1
        // Debug.Log($"{accel}, {steering}");
        // Input from network is between -1 to 1, map values accordingly
        // accel = accel * 2 - 1;
        continuousActionsOut[0] = accel;
        continuousActionsOut[1] = steering;
    }

}
