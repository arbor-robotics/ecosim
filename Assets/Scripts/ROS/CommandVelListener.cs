using UnityEngine;
using System.IO;
using System;
using std_msgs.msg;
using geometry_msgs.msg;
using sensor_msgs.msg;
using System.Linq;
using System.Threading.Tasks;
using Unity.Collections;
using System.Security.Cryptography;
using Unity.Mathematics;
using Unity.VisualScripting;

namespace ROS2
{
    public class CommandVelListener : MonoBehaviour
    {

        private WheelCollider[] wC;

        // Start is called before the first frame update
        private ROS2UnityComponent rosUnityComponent;
        private ROS2Node rosNode;
        private ISubscription<TwistStamped> commandVelSub;

        /// <summary>
        /// I added this stuff below from CarController script
        /// </summary>
        /// [SerializeField] private float mass = 1200.0f;
        /// 
        [SerializeField] private float mass = 1200.0f;
        [SerializeField] private Vector3 coG = new Vector3(0.0f, 0.435f, -2.5f);
        [SerializeField] private Vector3 inertiaTensor = new Vector3(3600.0f, 3900.0f, 800.0f);

        public Transform frWheelModel;


        public float GetVel { get { return vel; } }
        public float GetMass { get { return mass; } }
        public Vector3 GetCoG { get { return coG; } }
        public Rigidbody GetRB { get { return rB; } }

        private WheelCollider[] wC;
        private Rigidbody rB;
        private float vel;
        private float steerAngle = 0.0f;

        private AeroDynamics aeroDynamics;
        private Brakes brakes;
        private Engine engine;
        private Steering steering;
        private Suspension suspension;
        private Transmission transmission;
        private UserInput userInput;
        

        // TODO: Transform broadcaster

        PoseWithCovarianceStamped currentPose;
        NavSatFix currentFix;
        UnityEngine.Transform mapOrigin;

        public string nodeName;
        public string cmdTopic;

        void Start()
        {
            rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();

            // Find all WheelCollider objects as children of this object.
            wC = gameObject.GetComponentsInChildren<WheelCollider>();
        }

        void Update()
        {
            if (rosUnityComponent.Ok())
            {
                if (rosNode == null)
                {
                    // Set up the node and subscriber.
                    rosNode = rosUnityComponent.CreateNode(nodeName);
                    
                    commandVelSub =  rosNode.CreateSubscription<TwistStamped>(
                                    "/cmd_vel", commandVelCB);
                }   
            }
        }

       

        builtin_interfaces.msg.Time GetStamp()
        {
            builtin_interfaces.msg.Time stamp = new builtin_interfaces.msg.Time();

            float currentTime = Time.time;
            int secs = (int)math.floor(currentTime);
            uint nanos = (uint)((currentTime - secs) * 1e9);

            stamp.Sec = secs;
            stamp.Nanosec = nanos;

            return stamp;
        }

        void commandVelCB(TwistStamped msg){
            Debug.Log(msg.Twist.Linear.X); // m/s
            Debug.Log(msg.Twist.Angular.Z); // rad/s
            Debug.Log("Subscriber message");

            // wheels 0 1 2 3: RL RR FL FR
            // if linear x > 0 set y to 1 and list steps
            // if angular z negative or positive then adjust steering angle aaccordingly
            // wC[0].brakeTorque = something
            // wC[0].engineTorque = something
            // wC[0].steerAngle = something
            float inputX = 0.0f;
            float inputY = 0.0f;
            float inputH = 0.0f;


            if (msg.Twist.Linear.X >0){
                // input y steps
                 inputY = 1.0f;
            }
            // cw negative right, ccw positive left
            if (msg.Twist.Angular.Z > 0){
                inputX = -1.0f;
            }
            if (msg.Twist.Angular.Z < 0){
                inputX = 1.0f;
            }

            ///with inputs: inputX and inputY shoould be able to do all steps in madhusthas controller
            
            // calculate vehicle velocity in the forward direction
            vel = transform.InverseTransformDirection(rB.velocity).z;

            // update aerodynamic drag and lift
            aeroDynamics.ApplyAeroDrag(vel);
            aeroDynamics.ApplyAeroLift(vel);

            // update steering angle due to input, correct for ackermann and apply steering (if we have a steering script)
            steerAngle = steering.SteerAngle(vel, inputX, steerAngle);
            wC[2].steerAngle = steering.AckerAdjusted(steerAngle, suspension.GetWheelBase, suspension.GetTrackFront, true);
            wC[3].steerAngle = steering.AckerAdjusted(steerAngle, suspension.GetWheelBase, suspension.GetTrackFront, false);


            // update lateral load transfer from anti-roll bars (sway bars)
            suspension.ApplyLLT();

            // if automatic, select gear appropriate for vehicle speed (unless reverse requested)
            if (transmission.GetAutomatic)
            {
                if (Input.GetKey(KeyCode.R)) transmission.SelectReverse();
                else transmission.SetGear(suspension.GetNoSlipWheelRPM(vel), engine.GetEngineRPMMaxPower);
            }

            // update engine rpm and available torque
            float transmissionRatio = transmission.GetTransmissionRatio();
            float engineClutchLockRPM = transmission.GetEngineClutchLockRPM;
            engine.UpdateEngineSpeedRPM(suspension.GetNoSlipWheelRPM(vel), inputY, transmissionRatio, engineClutchLockRPM);
            float engineTorque = engine.GetMaxEngineTorque();

            // get requested engine torque
            if (inputY > 0.2f) engineTorque *= inputY;
            else engineTorque = 0.0f;

            // get requested wheel torques
            float[] wheelTorques = transmission.GetWheelTorques(engineTorque);

            // get traction control torque updates
            // if you want to add a traction control module, this would be a good place to use it

            // get requested brake torques
            float[] brakeTorques = brakes.GetBrakeTorques(inputY);

            // if handbrake is on, brake rear wheels
            if (inputH > 0.1f) brakes.ApplyHandbrake(brakeTorques);

            // Calculate a wheel rpm limit based on engine limit and transmission
            float wheelRPMLimit = Mathf.Abs(engine.GetEngineRPMMax / transmission.GetTransmissionRatio()) * 1.01f;

            // check if wheel should be hitting engine rpm limit, if so, cut power to prevent over revving of wheel
            int iDrivenWheelID;
            for (int j = 0; j < transmission.GetDrivenWheels.Count; j++)
            {
                iDrivenWheelID = transmission.GetDrivenWheels[j];
                if (wC[transmission.GetDrivenWheels[j]].rpm > wheelRPMLimit) wheelTorques[iDrivenWheelID] = 0.0f;
            }

            // update brake and motor torques on wheel colliders
            // RL RR FL FR
            // initialise all to 0.0f
            for (int i = 0; i < 4; i++)
            {
                wC[i].brakeTorque = brakeTorques[i];
                wC[i].motorTorque = wheelTorques[i];
            }
        }
    }
}  // namespace ROS2