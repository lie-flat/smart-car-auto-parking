using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimpleCarController : MonoBehaviour
{
    public List<AxleInfo> axleInfos; // the information about each individual axle
    public float maxMotorTorque; // maximum torque the motor can apply to wheel
    public float maxSteeringAngle; // maximum steer angle the wheel can have

    private float motorVal = 0, steeringVal = 0, motor = 0, steering = 0;

    public float Motor { get => motor; }
    public float Steering { get => steering; }

    public void Control(float steering, float motor)
    {
        this.motor = motor;
        this.steering = steering;
        motorVal = maxMotorTorque * motor;
        steeringVal = maxSteeringAngle * steering;
        // Debug.Log($"Steering: {steering}, motor: {motorVal}")
        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steeringVal;
                axleInfo.rightWheel.steerAngle = steeringVal;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motorVal;
                axleInfo.rightWheel.motorTorque = motorVal;
            }
        }
    }
}

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor; // is this wheel attached to motor?
    public bool steering; // does this wheel apply steer angle?
}