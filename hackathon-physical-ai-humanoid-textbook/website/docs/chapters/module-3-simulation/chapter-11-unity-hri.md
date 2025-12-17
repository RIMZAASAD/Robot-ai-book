---
title: "Chapter 11 - Unity for Human–Robot Interaction"
module: "Digital Twin Simulation"
chapter: 11
description: "Unity integration for human-robot interaction scenarios and simulation"
learningObjectives:
  - "Set up Unity for human-robot interaction simulation"
  - "Create interactive environments for HRI research"
  - "Implement VLA pipeline integration with Unity"
prerequisites: ["chapter-10-physics-sensor-simulation"]
difficulty: "intermediate"
---

## Learning Objectives

- Set up Unity for human-robot interaction simulation
- Create interactive environments for HRI research
- Implement VLA pipeline integration with Unity

## Introduction

Unity provides a powerful platform for creating realistic human-robot interaction (HRI) scenarios that complement traditional robotics simulation tools. While Gazebo excels at physics simulation, Unity offers superior visual fidelity and interactive capabilities for HRI research. This chapter explores the integration of Unity with ROS 2 for creating immersive human-robot interaction environments, supporting the Vision-Language-Action pipeline through realistic visual simulation and interactive scenarios. Unity's capabilities for creating human-centered environments align with our Anthropomorphic Focus principle, enabling the development of humanoid robots that can operate effectively in spaces designed for human use.

## Unity in the Robotics Ecosystem

### Unity vs. Traditional Robotics Simulation

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| Physics Simulation | High-fidelity, real-time | Good, with Unity Physics |
| Visual Fidelity | Moderate | Very High |
| Human-Centered Environments | Basic | Excellent |
| Interactivity | Limited | Extensive |
| Realism | Functional | Photorealistic |
| VR/AR Support | Limited | Excellent |

### Unity Robotics Hub

Unity provides the Robotics Hub package that facilitates integration with ROS 2:

- **ROS-TCP-Connector**: Communication bridge between Unity and ROS 2
- **URDF-Importer**: Import URDF models directly into Unity
- **Robotics-Object-Pools**: Optimized object management for robotics simulations
- **Visualizations**: Tools for sensor data visualization

## Setting Up Unity for Robotics

### Prerequisites and Installation

1. **Unity Hub**: Download from unity3d.com
2. **Unity Editor**: Install version 2021.3 LTS or later
3. **ROS 2**: Ensure ROS 2 (Humble Hawksbill) is installed
4. **Visual Studio**: For C# scripting (Windows) or appropriate IDE

### Installing Unity Robotics Packages

1. Open Unity Hub and create a new 3D project
2. In the Package Manager (Window > Package Manager):
   - Install "ROS TCP Connector" from Package Manager
   - Install "URDF Importer" from Package Manager
   - Install "Cinemachine" for camera control

### Basic Project Structure

```
UnityHRIProject/
├── Assets/
│   ├── Scripts/           # C# scripts for robotics integration
│   ├── URDFs/            # Imported robot models
│   ├── Scenes/           # Unity scenes for different environments
│   ├── Materials/        # Visual materials for realistic rendering
│   ├── Prefabs/          # Reusable robot and environment objects
│   └── Plugins/          # ROS communication libraries
├── Packages/
└── ProjectSettings/
```

## ROS-TCP-Connector Integration

### Setting up ROS Communication

First, install the ROS-TCP-Connector in Unity:

```csharp
// RobotCommunicator.cs - Basic ROS communication setup
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotCommunicator : MonoBehaviour
{
    ROSConnection ros;
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.instance;
        ros.RegisteredGID += OnRegisteredGID;

        // Set the IP address and port for the ROS connection
        ros.Initialize(rosIPAddress, rosPort);

        Debug.Log($"ROS Connection initialized to {rosIPAddress}:{rosPort}");
    }

    void OnRegisteredGID(ulong GID)
    {
        Debug.Log($"Connected to ROS with GID: {GID}");
    }

    void OnDestroy()
    {
        if (ros != null)
        {
            ros.RegisteredGID -= OnRegisteredGID;
        }
    }
}
```

### Publishing and Subscribing to ROS Topics

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class HumanoidController : MonoBehaviour
{
    ROSConnection ros;

    // Topics
    public string jointStatesTopic = "/joint_states";
    public string cameraTopic = "/camera/image_raw";

    // Robot joint transforms
    public Transform leftHip;
    public Transform leftKnee;
    public Transform leftAnkle;
    public Transform rightHip;
    public Transform rightKnee;
    public Transform rightAnkle;

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to joint states
        ros.Subscribe<JointStateMsg>(jointStatesTopic, JointStateCallback);
    }

    void JointStateCallback(JointStateMsg jointState)
    {
        // Update robot joint positions based on ROS messages
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            double jointPosition = jointState.position[i];

            Transform jointTransform = GetJointTransformByName(jointName);
            if (jointTransform != null)
            {
                // Apply rotation based on joint position
                jointTransform.localRotation = Quaternion.Euler(
                    (float)jointPosition * Mathf.Rad2Deg, 0, 0);
            }
        }
    }

    Transform GetJointTransformByName(string name)
    {
        switch (name)
        {
            case "left_hip_joint": return leftHip;
            case "left_knee_joint": return leftKnee;
            case "left_ankle_joint": return leftAnkle;
            case "right_hip_joint": return rightHip;
            case "right_knee_joint": return rightKnee;
            case "right_ankle_joint": return rightAnkle;
            default: return null;
        }
    }

    void Update()
    {
        // Send periodic status updates
        SendRobotStatus();
    }

    void SendRobotStatus()
    {
        // Create and publish robot status message
        // Implementation depends on specific requirements
    }
}
```

## Creating Human-Centered Environments

### Environment Design Principles

For effective HRI simulation, environments should reflect human-centered design:

1. **Proportional Accuracy**: Furniture and spaces at human scale
2. **Navigation Paths**: Clear pathways for both humans and robots
3. **Interaction Points**: Counters, tables, and surfaces for object exchange
4. **Accessibility Features**: Doors, ramps, and controls usable by humanoid robots

### Sample Kitchen Environment

```csharp
using UnityEngine;

public class KitchenEnvironment : MonoBehaviour
{
    [Header("Furniture References")]
    public Transform counterTop;
    public Transform fridge;
    public Transform table;
    public Transform cabinet;

    [Header("Interaction Points")]
    public Transform[] pickupPoints;
    public Transform[] placePoints;
    public Transform[] navigationWaypoints;

    void Start()
    {
        SetupEnvironment();
    }

    void SetupEnvironment()
    {
        // Configure counter height for humanoid interaction
        if (counterTop != null)
        {
            counterTop.position = new Vector3(0, 0.9f, 0); // Standard counter height
        }

        // Setup pickup and place points
        SetupInteractionPoints();

        // Create navigation mesh for pathfinding
        CreateNavigationMesh();
    }

    void SetupInteractionPoints()
    {
        // Define points where robot can interact with objects
        // These should be at appropriate heights for humanoid manipulation
        for (int i = 0; i < pickupPoints.Length; i++)
        {
            // Ensure points are reachable by humanoid arm
            if (pickupPoints[i].position.y < 0.6f || pickupPoints[i].position.y > 1.2f)
            {
                Debug.LogWarning($"Pickup point {i} may be unreachable for humanoid");
            }
        }
    }

    void CreateNavigationMesh()
    {
        // In practice, you'd use Unity's NavMesh system
        // This is a placeholder for navigation setup
        Debug.Log("Navigation mesh setup completed");
    }
}
```

### Lighting and Realism

For photorealistic HRI scenarios:

```csharp
using UnityEngine;
using UnityEngine.Rendering;

public class EnvironmentLighting : MonoBehaviour
{
    [Header("Lighting Setup")]
    public Light mainLight;
    public Light[] fillLights;
    public bool useHDRP = false;

    void Start()
    {
        ConfigureLighting();
    }

    void ConfigureLighting()
    {
        if (useHDRP)
        {
            ConfigureHDRPLighting();
        }
        else
        {
            ConfigureBuiltInLighting();
        }
    }

    void ConfigureHDRPLighting()
    {
        // Configure High Definition Render Pipeline
        RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;
    }

    void ConfigureBuiltInLighting()
    {
        // Configure built-in render pipeline
        if (mainLight != null)
        {
            mainLight.type = LightType.Directional;
            mainLight.intensity = 1.2f;
            mainLight.color = Color.white;
            mainLight.shadows = LightShadows.Soft;
        }

        // Add ambient lighting
        RenderSettings.ambientLight = new Color(0.4f, 0.4f, 0.4f, 1);
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
    }

    void OnBeginCameraRendering(ScriptableRenderContext context, Camera camera)
    {
        // HDRP-specific rendering setup
    }
}
```

## Implementing Human-Robot Interaction Scenarios

### Basic HRI Framework

```csharp
using UnityEngine;
using System.Collections.Generic;

public class HRIManager : MonoBehaviour
{
    [Header("Human Interaction")]
    public GameObject humanCharacter;
    public float interactionDistance = 2.0f;

    [Header("Robot Configuration")]
    public GameObject humanoidRobot;
    public Transform[] interactionZones;

    [Header("Communication")]
    public string speechTopic = "/speech_recognition";
    public string gestureTopic = "/gesture_detection";

    private List<InteractionEvent> activeInteractions = new List<InteractionEvent>();

    void Update()
    {
        CheckForInteractions();
        UpdateInteractionStatus();
    }

    void CheckForInteractions()
    {
        if (humanCharacter == null || humanoidRobot == null) return;

        float distance = Vector3.Distance(
            humanCharacter.transform.position,
            humanoidRobot.transform.position
        );

        if (distance <= interactionDistance)
        {
            // Trigger interaction logic
            HandleProximityInteraction();
        }
    }

    void HandleProximityInteraction()
    {
        // Implement interaction logic based on distance and context
        Debug.Log("Human-Robot interaction detected!");

        // Possible interactions:
        // - Greeting sequence
        // - Task assignment
        // - Object handover
        // - Navigation assistance
    }

    void UpdateInteractionStatus()
    {
        // Update UI elements, send ROS messages, etc.
    }
}

[System.Serializable]
public class InteractionEvent
{
    public string eventType;
    public float timestamp;
    public Vector3 humanPosition;
    public Vector3 robotPosition;
    public string context;
}
```

### Gesture Recognition and Response

```csharp
using UnityEngine;

public class GestureRecognition : MonoBehaviour
{
    [Header("Gesture Configuration")]
    public Transform humanHandLeft;
    public Transform humanHandRight;
    public float gestureThreshold = 0.1f;

    [Header("Gesture Responses")]
    public GameObject robotHead;
    public float headTurnSpeed = 2.0f;

    private Vector3 lastLeftHandPos;
    private Vector3 lastRightHandPos;
    private bool gestureDetected = false;

    void Start()
    {
        if (humanHandLeft != null) lastLeftHandPos = humanHandLeft.position;
        if (humanHandRight != null) lastRightHandPos = humanHandRight.position;
    }

    void Update()
    {
        DetectGestures();
        RespondToGestures();
    }

    void DetectGestures()
    {
        if (humanHandLeft == null || humanHandRight == null) return;

        Vector3 currentLeftPos = humanHandLeft.position;
        Vector3 currentRightPos = humanHandRight.position;

        // Calculate movement vectors
        Vector3 leftMovement = currentLeftPos - lastLeftHandPos;
        Vector3 rightMovement = currentRightPos - lastRightHandPos;

        // Detect specific gestures
        if (leftMovement.magnitude > gestureThreshold)
        {
            DetectLeftHandGesture(leftMovement);
        }

        if (rightMovement.magnitude > gestureThreshold)
        {
            DetectRightHandGesture(rightMovement);
        }

        // Update positions for next frame
        lastLeftHandPos = currentLeftPos;
        lastRightHandPos = currentRightPos;
    }

    void DetectLeftHandGesture(Vector3 movement)
    {
        // Example: Wave gesture (horizontal movement)
        if (Mathf.Abs(movement.x) > Mathf.Abs(movement.y) &&
            Mathf.Abs(movement.x) > Mathf.Abs(movement.z))
        {
            if (movement.x > 0)
            {
                Debug.Log("Left hand wave right detected");
                RespondToWave();
            }
            else
            {
                Debug.Log("Left hand wave left detected");
            }
        }
    }

    void DetectRightHandGesture(Vector3 movement)
    {
        // Example: Point gesture (forward movement)
        if (movement.z > gestureThreshold * 2)
        {
            Debug.Log("Pointing gesture detected");
            LookAtPoint();
        }
    }

    void RespondToWave()
    {
        // Robot responds to wave with head movement or speech
        StartCoroutine(RobotWaveResponse());
    }

    void LookAtPoint()
    {
        // Robot looks in the direction pointed by human
        if (robotHead != null)
        {
            Vector3 lookDirection = (humanHandRight.position - transform.position).normalized;
            Quaternion targetRotation = Quaternion.LookRotation(lookDirection, Vector3.up);
            robotHead.rotation = Quaternion.Slerp(
                robotHead.rotation,
                targetRotation,
                headTurnSpeed * Time.deltaTime
            );
        }
    }

    System.Collections.IEnumerator RobotWaveResponse()
    {
        // Simple head nod response
        if (robotHead != null)
        {
            Vector3 originalRotation = robotHead.localEulerAngles;
            for (float t = 0; t < 1; t += Time.deltaTime * 2)
            {
                robotHead.localRotation = Quaternion.Euler(
                    originalRotation.x + Mathf.Sin(t * Mathf.PI * 4) * 10,
                    originalRotation.y,
                    originalRotation.z
                );
                yield return null;
            }
            robotHead.localEulerAngles = originalRotation;
        }
    }
}
```

## Vision-Language-Action Pipeline Integration

### Unity Camera Integration

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections;

public class UnityCameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string cameraTopic = "/unity_camera/image_raw";
    public Camera unityCamera;
    public int imageWidth = 640;
    int imageHeight = 480;
    int publishRate = 30; // Hz

    RenderTexture renderTexture;
    Texture2D texture2D;

    void Start()
    {
        ros = ROSConnection.instance;

        // Create render texture for camera
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        if (unityCamera != null)
        {
            unityCamera.targetTexture = renderTexture;
        }

        // Start coroutine to publish images at desired rate
        StartCoroutine(PublishCameraImages());
    }

    IEnumerator PublishCameraImages()
    {
        float frameTime = 1.0f / publishRate;

        while (true)
        {
            yield return new WaitForSeconds(frameTime);
            PublishImage();
        }
    }

    void PublishImage()
    {
        if (unityCamera == null) return;

        // Create temporary render texture to read pixels
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;

        if (texture2D == null)
        {
            texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        }

        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();

        // Convert to ROS image format
        ImageMsg rosImage = CreateROSImage(texture2D);

        // Publish to ROS
        ros.Publish(cameraTopic, rosImage);

        RenderTexture.active = currentRT;
    }

    ImageMsg CreateROSImage(Texture2D texture)
    {
        // Convert Unity texture to ROS Image message
        ImageMsg img = new ImageMsg();
        img.header = new std_msgs.Header();
        img.header.stamp = new builtin_interfaces.Time();
        img.header.frame_id = "unity_camera_optical_frame";

        img.height = (uint)texture.height;
        img.width = (uint)texture.width;
        img.encoding = "rgb8";
        img.is_bigendian = false;
        img.step = (uint)(texture.width * 3); // 3 bytes per pixel for RGB

        // Convert texture to byte array
        Color32[] colors = texture.GetPixels32();
        byte[] imageData = new byte[colors.Length * 3];

        for (int i = 0; i < colors.Length; i++)
        {
            imageData[i * 3] = colors[i].r;
            imageData[i * 3 + 1] = colors[i].g;
            imageData[i * 3 + 2] = colors[i].b;
        }

        img.data = imageData;
        return img;
    }
}
```

### Speech and Language Integration

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Diagnostic;

public class SpeechIntegration : MonoBehaviour
{
    ROSConnection ros;
    public string speechCommandTopic = "/speech_commands";
    public string textToSpeechTopic = "/tts_input";

    [Header("Speech Configuration")]
    public float confidenceThreshold = 0.7f;
    public string[] validCommands = {
        "hello", "stop", "go", "pick up", "put down", "follow me"
    };

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to speech recognition results
        ros.Subscribe<StringMsg>("/speech_recognition", SpeechCallback);
    }

    void SpeechCallback(StringMsg speechMsg)
    {
        string recognizedText = speechMsg.data;
        Debug.Log($"Speech recognized: {recognizedText}");

        ProcessSpeechCommand(recognizedText);
    }

    void ProcessSpeechCommand(string command)
    {
        // Check if command is in our valid command list
        foreach (string validCmd in validCommands)
        {
            if (command.ToLower().Contains(validCmd.ToLower()))
            {
                ExecuteCommand(validCmd);
                return;
            }
        }

        // If no valid command found, publish to TTS
        StringMsg ttsMsg = new StringMsg();
        ttsMsg.data = $"I don't understand the command: {command}";
        ros.Publish(textToSpeechTopic, ttsMsg);
    }

    void ExecuteCommand(string command)
    {
        Debug.Log($"Executing command: {command}");

        // Publish command-specific messages to ROS
        StringMsg cmdMsg = new StringMsg();
        cmdMsg.data = command;
        ros.Publish(speechCommandTopic, cmdMsg);

        // Provide feedback
        StringMsg feedbackMsg = new StringMsg();
        feedbackMsg.data = $"Okay, I will {command}";
        ros.Publish(textToSpeechTopic, feedbackMsg);
    }

    // Method to simulate speech input (for testing)
    public void SimulateSpeechInput(string text)
    {
        StringMsg speechMsg = new StringMsg();
        speechMsg.data = text;
        SpeechCallback(speechMsg);
    }
}
```

## Performance Optimization for Real-time HRI

### Object Pooling for Dynamic Objects

```csharp
using UnityEngine;
using System.Collections.Generic;

public class ObjectPooler : MonoBehaviour
{
    [System.Serializable]
    public class Pool
    {
        public string tag;
        public GameObject prefab;
        public int size;
    }

    public List<Pool> pools;
    public Dictionary<string, Queue<GameObject>> poolDictionary;

    void Start()
    {
        poolDictionary = new Dictionary<string, Queue<GameObject>>();

        foreach (Pool pool in pools)
        {
            Queue<GameObject> objectPool = new Queue<GameObject>();

            for (int i = 0; i < pool.size; i++)
            {
                GameObject obj = Instantiate(pool.prefab);
                obj.SetActive(false);
                objectPool.Enqueue(obj);
            }

            poolDictionary.Add(pool.tag, objectPool);
        }
    }

    public GameObject SpawnFromPool(string tag, Vector3 position, Quaternion rotation)
    {
        if (!poolDictionary.ContainsKey(tag))
        {
            Debug.LogWarning($"Pool with tag {tag} doesn't exist!");
            return null;
        }

        GameObject objectToSpawn = poolDictionary[tag].Dequeue();
        objectToSpawn.SetActive(true);
        objectToSpawn.transform.position = position;
        objectToSpawn.transform.rotation = rotation;

        // Add object back to pool when deactivated
        PoolObject poolObj = objectToSpawn.GetComponent<PoolObject>();
        if (poolObj == null)
        {
            poolObj = objectToSpawn.AddComponent<PoolObject>();
        }
        poolObj.pool = this;
        poolObj.tag = tag;

        poolDictionary[tag].Enqueue(objectToSpawn);
        return objectToSpawn;
    }
}

public class PoolObject : MonoBehaviour
{
    public ObjectPooler pool;
    public string tag;

    void OnDisable()
    {
        if (pool != null)
        {
            pool.SpawnFromPool(tag, transform.position, transform.rotation);
        }
    }
}
```

### Level of Detail (LOD) for Complex Environments

```csharp
using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
public class HRILODSystem : MonoBehaviour
{
    [Header("LOD Configuration")]
    public float[] lodDistances = { 10f, 30f, 50f };
    public Mesh[] lodMeshes;

    [Header("Performance Settings")]
    public bool enableLOD = true;
    public float updateInterval = 0.1f;

    private MeshRenderer meshRenderer;
    private Transform cameraTransform;
    private float lastUpdateTime;
    private int currentLOD = 0;

    void Start()
    {
        meshRenderer = GetComponent<MeshRenderer>();
        cameraTransform = Camera.main.transform;

        if (lodMeshes.Length == 0)
        {
            // If no LOD meshes provided, disable LOD
            enableLOD = false;
            return;
        }
    }

    void Update()
    {
        if (!enableLOD || cameraTransform == null) return;

        if (Time.time - lastUpdateTime > updateInterval)
        {
            UpdateLOD();
            lastUpdateTime = Time.time;
        }
    }

    void UpdateLOD()
    {
        if (lodMeshes.Length == 0) return;

        float distance = Vector3.Distance(transform.position, cameraTransform.position);

        int newLOD = 0;
        for (int i = 0; i < lodDistances.Length; i++)
        {
            if (distance > lodDistances[i])
            {
                newLOD = i + 1;
            }
            else
            {
                break;
            }
        }

        // Clamp to available LOD levels
        newLOD = Mathf.Min(newLOD, lodMeshes.Length - 1);

        if (newLOD != currentLOD)
        {
            UpdateMesh(newLOD);
            currentLOD = newLOD;
        }
    }

    void UpdateMesh(int lodLevel)
    {
        if (lodLevel < lodMeshes.Length)
        {
            MeshFilter meshFilter = GetComponent<MeshFilter>();
            if (meshFilter != null)
            {
                meshFilter.mesh = lodMeshes[lodLevel];
            }
        }
    }
}
```

## Constitution Alignment

This chapter addresses several constitutional requirements:

### Anthropomorphic Focus (Principle II)
- Human-centered environment design for humanoid robot interaction
- Proportional accuracy for human-sized furniture and spaces
- Interaction scenarios designed for human-robot collaboration

### Sim-to-Real Rigor (Principle III)
- Realistic visual simulation for vision system training
- Proper sensor simulation integration with Unity cameras
- Environment complexity matching real-world scenarios

### Visualization Requirements (Key Standard II)
- High-fidelity visual rendering for realistic HRI scenarios
- Proper material and lighting setup for photorealistic environments
- Clear examples with proper code formatting

### VLA Convergence Mandate (Principle I)
- Integration of vision, language, and action systems in Unity
- Camera integration for vision processing
- Speech and command processing for language interface

## Practical Examples

### Example 1: Humanoid Assistant Scenario

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Actionlib;

public class HumanoidAssistant : MonoBehaviour
{
    ROSConnection ros;
    public Transform[] servicePoints;
    public float serviceRadius = 2.0f;

    [Header("Service Actions")]
    public string navigationTopic = "/move_base_simple/goal";
    public string manipulationTopic = "/manipulation_controller/command";

    void Start()
    {
        ros = ROSConnection.instance;
        Debug.Log("Humanoid assistant initialized");
    }

    void Update()
    {
        CheckForServiceRequests();
    }

    void CheckForServiceRequests()
    {
        // Check if human is within service radius of any service point
        foreach (Transform servicePoint in servicePoints)
        {
            Collider[] hitColliders = Physics.OverlapSphere(servicePoint.position, serviceRadius);
            foreach (Collider collider in hitColliders)
            {
                if (collider.CompareTag("Human"))
                {
                    HandleServiceRequest(servicePoint, collider.gameObject);
                    break;
                }
            }
        }
    }

    void HandleServiceRequest(Transform servicePoint, GameObject human)
    {
        Debug.Log($"Service request detected at {servicePoint.name}");

        // Move to service point
        MoveToLocation(servicePoint.position);

        // Wait for service completion
        StartCoroutine(ProvideService(servicePoint, human));
    }

    void MoveToLocation(Vector3 targetPosition)
    {
        // Publish navigation goal to ROS
        // Implementation would involve sending a PoseStamped message
        Debug.Log($"Moving to {targetPosition}");
    }

    System.Collections.IEnumerator ProvideService(Transform servicePoint, GameObject human)
    {
        yield return new WaitForSeconds(2.0f); // Simulate service time

        // Publish service completion
        Debug.Log("Service completed");
    }
}
```

### Example 2: Collaborative Task Environment

```csharp
using UnityEngine;
using System.Collections.Generic;

public class CollaborativeTaskManager : MonoBehaviour
{
    [Header("Task Configuration")]
    public List<TaskDefinition> tasks;
    public Transform[] workstations;
    public GameObject[] objectsToManipulate;

    [Header("Human-Robot Collaboration")]
    public float collaborationDistance = 1.5f;
    public float taskCompletionThreshold = 0.1f;

    private int currentTaskIndex = 0;
    private bool taskInProgress = false;

    void Update()
    {
        if (tasks.Count > 0 && !taskInProgress)
        {
            StartNextTask();
        }
    }

    void StartNextTask()
    {
        if (currentTaskIndex >= tasks.Count) return;

        TaskDefinition currentTask = tasks[currentTaskIndex];
        Debug.Log($"Starting task: {currentTask.taskName}");

        // Set up the task environment
        SetupTaskEnvironment(currentTask);

        taskInProgress = true;
    }

    void SetupTaskEnvironment(TaskDefinition task)
    {
        // Position objects according to task requirements
        for (int i = 0; i < task.objectPositions.Count; i++)
        {
            if (i < objectsToManipulate.Length)
            {
                objectsToManipulate[i].transform.position = task.objectPositions[i];
            }
        }

        // Enable task-specific interactions
        EnableTaskInteractions(task);
    }

    void EnableTaskInteractions(TaskDefinition task)
    {
        // Enable specific interaction modes based on task
        switch (task.taskType)
        {
            case TaskType.Assembly:
                EnableAssemblyMode();
                break;
            case TaskType.Transport:
                EnableTransportMode();
                break;
            case TaskType.Inspection:
                EnableInspectionMode();
                break;
        }
    }

    void EnableAssemblyMode()
    {
        Debug.Log("Assembly mode enabled - human and robot can collaborate on assembly tasks");
    }

    void EnableTransportMode()
    {
        Debug.Log("Transport mode enabled - robot can assist with object transport");
    }

    void EnableInspectionMode()
    {
        Debug.Log("Inspection mode enabled - robot can assist with quality inspection");
    }
}

[System.Serializable]
public class TaskDefinition
{
    public string taskName;
    public TaskType taskType;
    public List<Vector3> objectPositions;
    public List<Transform> targetPositions;
    public string completionCriteria;
}

public enum TaskType
{
    Assembly,
    Transport,
    Inspection,
    Maintenance
}
```

## Exercises

### Exercise 1: HRI Environment Creation
Create a Unity scene that includes:
- A human-centered environment (e.g., kitchen, office, or living room)
- Properly scaled furniture for humanoid robot interaction
- Interactive elements for human-robot collaboration
- Realistic lighting and materials

### Exercise 2: ROS Integration
Implement ROS communication in Unity that:
- Subscribes to joint states and visualizes robot in Unity
- Publishes camera images from Unity to ROS
- Integrates speech recognition and text-to-speech
- Handles basic navigation and manipulation commands

### Exercise 3: VLA Pipeline Integration
Create a complete Unity scene that demonstrates:
- Vision system integration with Unity cameras
- Language processing for human commands
- Action execution for robot behaviors
- Real-time interaction between human and robot avatars

## Summary

Unity provides a powerful platform for creating realistic human-robot interaction scenarios that complement traditional robotics simulation. The high visual fidelity and interactive capabilities of Unity make it ideal for developing and testing humanoid robots in human-centered environments. Integration with ROS 2 enables the full Vision-Language-Action pipeline to be tested in photorealistic settings, supporting the development of robots that can operate effectively in spaces designed for human use.

## Further Reading

- "Unity Robotics Package Documentation" - Official Unity Robotics Hub guide
- "Human-Robot Interaction: Fundamentals and Implementation" by Chen and Sauser
- "Unity in Action" by Joe Hocking (for Unity fundamentals)
- "ROS Robotics Projects" by Anil Mahtani (for ROS-Unity integration)
- "Computer Graphics: Principles and Practice" for realistic rendering