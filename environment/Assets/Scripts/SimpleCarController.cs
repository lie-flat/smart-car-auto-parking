using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimpleCarController : MonoBehaviour
{
    public List<AxleInfo> axleInfos; // the information about each individual axle
    public float maxMotorTorque; // maximum torque the motor can apply to wheel
    public float maxSteeringAngle; // maximum steer angle the wheel can have

    public void Control(float steering, float motor)
    {
        float motorVal = maxMotorTorque * motor;
        float steeringVal = maxSteeringAngle * steering;
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