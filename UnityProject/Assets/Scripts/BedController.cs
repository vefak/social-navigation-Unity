using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.UrdfImporter.Control;
using System;

namespace RosSharp.Control
{

    public class BedController : MonoBehaviour
    {
        public GameObject wheelFrontLeft;
        public GameObject wheelFrontRight;
        public GameObject wheelBackLeft;
        public GameObject wheelBackRight;
        public GameObject steerFrontLeft;
        public GameObject steerFrontRight;
        

        private ArticulationBody wheelFrontLeftA;  // Should be "wheel_front_left_link" in the script parameter
        private ArticulationBody wheelFrontRightA;  // Should be "wheel_front_right_link" in the script parameter
        private ArticulationBody wheelBackLeftA;  // Should be "wheel_back_left_link" in the script parameter
        private ArticulationBody wheelBackRightA;  // Should be "wheel_back_left_link" in the script parameter
        private ArticulationBody steerFrontLeftA;  // Should be "wheel_front_left_rot_link" in the script parameter
        private ArticulationBody steerFrontRightA;  // Should be "wheel_front_right_rot_link" in the script parameter

        public float maxLinearSpeed = 2;  // Maximum linear speed in forward direction in m/s
        public float minLinearSpeed = -2;  // Minimum linear speed in forward direction in m/s (Note: Should be negative)
        public float maxRotationalSpeed = 1;  // Maximal rotational speed in radians per second
        public float minRotationalSpeed = -1;  // Maximal rotational speed in radians per second (Note: Should be negative)

        public float wheelRadius = 0.04f;  // Radius of the 4 wheels in meters (Note: It is assumed, that all wheels have the same radius)
        public float trackWidth = 0.32f;  // Distance in meters between the back (and front) tyres
        public float wheelBase = 0.76f;  // Distance between the back and front axle in meters
        public float forceLimit = 10000000.0f;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        private float minAngularVelocity = 0.01f;

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        private bool collisionDetected = false;  // Flag to track collision
        // public bool isNewPathGenerated = false;  // Flag to track new path 

        void Start()
        {
            wheelFrontLeftA = wheelFrontLeft.GetComponent<ArticulationBody>();
            wheelFrontRightA = wheelFrontRight.GetComponent<ArticulationBody>();
            wheelBackLeftA = wheelBackLeft.GetComponent<ArticulationBody>();
            wheelBackRightA = wheelBackRight.GetComponent<ArticulationBody>();
            steerFrontLeftA = steerFrontLeft.GetComponent<ArticulationBody>();
            steerFrontRightA = steerFrontRight.GetComponent<ArticulationBody>();

            SetParameters(wheelFrontLeftA);
            SetParameters(wheelFrontRightA);
            SetParameters(wheelBackLeftA);
            SetParameters(wheelBackRightA);
            SetParameters(steerFrontLeftA);
            SetParameters(steerFrontRightA);

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSVelCmd);
            // ros.Subscribe<BoolMsg>("is_new_path_generated", ReceiveNewPath);

            // Subscribe to collision detection events
            //CollisionDetection.OnCollisionDetected += HandleCollisionDetected;
            //CollisionDetection.OnCollisionEnded += HandleCollisionEnded;

        }

        void ReceiveROSVelCmd(TwistMsg cmdVel)
        {    
            if (!collisionDetected) // Only update commands if no collision or new path generated
            {
                rosLinear = (float)cmdVel.linear.x;
                rosAngular = (float)cmdVel.angular.z;
                lastCmdReceived = Time.time;
            }
        }

        // void ReceiveNewPath(BoolMsg newPathMsg)
        // {
        //     // Handle new path command as necessary
        //     Debug.Log("New path generated status received.");
        //     // Assign the new path message to the public variable
        //     isNewPathGenerated = newPathMsg.data;
        // }

        void FixedUpdate()
        {
            if (!collisionDetected) // Only update commands if no collision
            {
                ROSUpdate();
            }
            else
            {
                StopMovement();
            }
        }

        // private void OnDestroy()
        // {
        //     // Unsubscribe from events to prevent memory leaks
        //     CollisionDetection.OnCollisionDetected -= HandleCollisionDetected;
        //     CollisionDetection.OnCollisionEnded -= HandleCollisionEnded;
        // }

        //private void HandleCollisionDetected()
        //{
        //    // Define behavior upon collision detection
        //    Debug.Log("Collision detected! Stopping robot movement. -- BedController.cs.");
        //    collisionDetected = true;
        //    StopMovement();
        //}
        //private void HandleCollisionEnded()
        //{
        //    Debug.Log("Collision ended. Resuming robot movement. Handling in BedController.cs");
        //    collisionDetected = false;
        //}

        private void StopMovement()
        {
         // Set the speed of the wheels and steering axis to 0
            SetSpeed(wheelFrontLeftA, 0);
            SetSpeed(wheelFrontRightA, 0);
            SetSpeed(wheelBackLeftA, 0);
            SetSpeed(wheelBackRightA, 0);
            SetAngle(steerFrontLeftA, 0);
            SetAngle(steerFrontRightA, 0);

        }
        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = 0.0f;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        private void SetAngle(ArticulationBody joint, float angle)
        {
            ArticulationDrive drive = joint.xDrive;
            joint.jointFriction = 10.0f;
            joint.angularDamping = 10.0f;
            drive.forceLimit = 10000.0f;
            drive.stiffness = 100000.0f;
            drive.damping = 10000.0f;
            drive.target = -angle;
            drive.targetVelocity = 100.0f;
            joint.xDrive = drive;
        }

        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            RobotInput(rosLinear, -rosAngular);
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (speed < minLinearSpeed && speed != 0)
            {
                speed = minLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }

            if (rotSpeed < minRotationalSpeed && rotSpeed != 0)
            {
                rotSpeed = minRotationalSpeed;
            }
            

            // Set wheel speed and steering parameters per default to drive a straight line
            float wheelFrontLeftRotation = (speed / wheelRadius);
            float wheelFrontRightRotation = wheelFrontLeftRotation;
            float wheelBackLeftRotation = wheelFrontLeftRotation;
            float wheelBackRightRotation = wheelFrontLeftRotation;
            float steerFrontLeftRotation = 0.0f;
            float steerFrontRightRotation = 0.0f;

            // Check if angluar velocity exceeds the minAngularVelocity-value and we should drive a turn instead of a straight line
            if (Math.Abs(rotSpeed) > minAngularVelocity)
            {
                float turnRadius = Math.Abs(speed / rotSpeed);
                float turnRadiusBackWheelLeft = 0.0f;
                float turnRadiusBackWheelRight = 0.0f;
                float turnRadiusFrontWheelLeft = 0.0f;
                float turnRadiusFrontWheelRight = 0.0f;
            
                // Check direction of turn to calculate the steering angles
                if (rotSpeed >= 0)
                {
                    // Counter-clockwise rotation
                    // Calculate the turning radius (distance of the ICR point to the wheel) for each wheel
                    turnRadiusBackWheelLeft = turnRadius - trackWidth / 2.0f;
                    turnRadiusBackWheelRight = turnRadius + trackWidth / 2.0f;
                    turnRadiusFrontWheelLeft = (float) Math.Sqrt((Math.Pow(turnRadiusBackWheelLeft, 2) + Math.Pow(wheelBase, 2)));
                    turnRadiusFrontWheelRight = (float) Math.Sqrt((Math.Pow(turnRadiusBackWheelRight, 2) + Math.Pow(wheelBase, 2)));
                }
                else
                {
                    // Clockwise rotation
                    // Calculate the turning radius (distance of the ICR point to the wheel) for each wheel
                    turnRadiusBackWheelLeft = turnRadius + trackWidth / 2.0f;
                    turnRadiusBackWheelRight = turnRadius - trackWidth / 2.0f;
                    turnRadiusFrontWheelLeft = (float) Math.Sqrt((Math.Pow(turnRadiusBackWheelLeft, 2) + Math.Pow(wheelBase, 2)));
                    turnRadiusFrontWheelRight = (float) Math.Sqrt((Math.Pow(turnRadiusBackWheelRight, 2) + Math.Pow(wheelBase, 2)));
                }

                // Calculate the front steering angles using their turning radiuses
                steerFrontLeftRotation = (float) Math.Atan2(wheelBase, turnRadiusBackWheelLeft) * Mathf.Rad2Deg * Math.Sign(rotSpeed) * Math.Sign(speed);
                steerFrontRightRotation = (float) Math.Atan2(wheelBase, turnRadiusBackWheelRight) * Mathf.Rad2Deg * Math.Sign(rotSpeed) * Math.Sign(speed);
                // Calculate the angle of the turn for one second using the linear velocity and the position in the middle of the back wheels [in radians per second]
                float drivenAngle = (float) (speed / turnRadius);
                // Calculate the circular arc of the wheels given their turning radiuses [length of circle arc per second]
                float arcCircleFrontWheelLeft = (float) Math.Abs(turnRadiusFrontWheelLeft * drivenAngle);
                float arcCircleFrontWheelRight = (float) Math.Abs(turnRadiusFrontWheelRight * drivenAngle);
                float arcCircleBackWheelLeft = (float) Math.Abs(turnRadiusBackWheelLeft * drivenAngle);
                float arcCircleBackWheelRight = (float) Math.Abs(turnRadiusBackWheelRight * drivenAngle);
                 // Calculate the wheel rotations for driving the calculated circle arc
                wheelFrontLeftRotation =  Math.Sign(speed) * (arcCircleFrontWheelLeft / wheelRadius) * Mathf.Rad2Deg;
                wheelFrontRightRotation = Math.Sign(speed) * (arcCircleFrontWheelRight / wheelRadius) * Mathf.Rad2Deg;
                wheelBackLeftRotation = Math.Sign(speed) * (arcCircleBackWheelLeft / wheelRadius) * Mathf.Rad2Deg;
                wheelBackRightRotation = Math.Sign(speed) * (arcCircleBackWheelRight / wheelRadius) * Mathf.Rad2Deg;

            }
            else
            {
                // Convert radians to degree
                wheelFrontLeftRotation *=  Mathf.Rad2Deg;
                wheelFrontRightRotation *=  Mathf.Rad2Deg;
                wheelBackLeftRotation *=  Mathf.Rad2Deg;
                wheelBackRightRotation *=  Mathf.Rad2Deg;
            }

            // Set the speed of the wheels and steering axis
            SetSpeed(wheelFrontLeftA, wheelFrontLeftRotation);
            SetSpeed(wheelFrontRightA, wheelFrontRightRotation);
            SetSpeed(wheelBackLeftA, wheelBackLeftRotation);
            SetSpeed(wheelBackRightA, wheelBackRightRotation);
            SetAngle(steerFrontLeftA, steerFrontLeftRotation);
            SetAngle(steerFrontRightA, steerFrontRightRotation);
        }
    }
}
