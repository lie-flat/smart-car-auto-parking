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

    public uint MaxSteps = 10000;
    private void CruiseControl(float steering, float motor)
    {
        var deltaMotor = carController.Motor < motor ? 0.1f : -0.1f;
        var deltaSteering = carController.Steering < steering ? 0.1f : -0.1f;

        carController.Control(carController.Steering + deltaSteering, carController.Motor + deltaMotor);

    }
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
        // transform.localPosition = new Vector3(
        //     // Spawn on 80% center area of the map
        //     0.8f * (Random.value - 0.5f) * Parameters.MAP_WIDTH,
        //     0.04f,
        //     0.8f * (Random.value - 0.5f) * Parameters.MAP_HEIGHT
        // );
        // transform.rotation = Quaternion.Euler(0, Random.value * 360, 0);
        // Determinastic spawn
        transform.localPosition = new Vector3(0.25f * Parameters.MAP_WIDTH, 0.1f, 0.3f * Parameters.MAP_HEIGHT);
        transform.localRotation = Quaternion.Euler(0, 90, 0);
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent positions
        sensor.AddObservation(transform.localPosition.x);
        sensor.AddObservation(transform.localPosition.z);

        // Agent rotation
        sensor.AddObservation(Mathf.Sin(transform.localEulerAngles.y));
        sensor.AddObservation(Mathf.Cos(transform.localEulerAngles.y));

        // Agent velocity
        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);

        // Agent Angular velocity
        sensor.AddObservation(rBody.angularVelocity.y);
        // Steering & Motor
        sensor.AddObservation(carController.Steering);
        sensor.AddObservation(carController.Motor);
    }

    public void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "Wall")
        {
            Debug.Log("Pong!");
            AddReward(-2f);
            // EpisodeInterrupted();
            // EndEpisode();
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var values = actions.ContinuousActions;
        var steering = values[0];
        var motor = values[1];
        // Debug.Log($"{steering}, {motor}");
        // CruiseControl(steering, motor);
        carController.Control(steering, motor);
        distance = Mathf.Sqrt(
            Mathf.Pow(transform.localPosition.x - targetX, 2f) +
            Mathf.Pow(transform.localPosition.z - targetZ, 2f));
        if (StepCount > 10000)
        {
            // MaxStep exceeded!
            SetReward(-1f);
            Debug.Log($"摸鱼警告!, CR: {GetCumulativeReward()}");
            EpisodeInterrupted();
        }
        else if (distance < 0.05)
        {
            // var angleDiff = Mathf.Abs(transform.localEulerAngles.y - (-30));
            // Reward is 2^(-deltaTheta)
            // Thus when deltaTheta -> 0, reward -> 1
            //      when deltaTheta >> 0, reward -> 0
            // angleDiff = Mathf.Deg2Rad * angleDiff;
            // AddReward(f);
            // var finalReward = 100 * Mathf.Pow(2, -angleDiff / 10);
            var finalReward = 100f;// * (360f - angleDiff) / 360f;
            AddReward(finalReward);
            Debug.LogWarning($"卷起来了, 花了 {StepCount} 步，角度{transform.localEulerAngles.y}，收益{GetCumulativeReward()}！最终激励：{finalReward}");
            EndEpisode();
        }
        else if (distance < 0.3666)
        {
            var reward = Mathf.Pow(2, -50 * distance);
            var discount = (MaxSteps - StepCount) / (float)MaxSteps;
            AddReward(discount * reward);
        }
        else if (transform.localPosition.y < 0.05)
        {
            // Fall
            SetReward(-1f);
            EndEpisode();
        }
        else
        {
            var disReward = -0.01f * distance;
            var stepReward = -Mathf.Pow(2, (StepCount - MaxSteps) / 1000);
            AddReward(Mathf.Min(disReward, stepReward));
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
