# Simplified drone flowchart

```mermaid
flowchart LR
    SENSORS["Drone sensors<br/>RGB-D camera + thermal camera + gyro"]
    PI["Raspberry Pi<br/>capture and synchronize sensors"]
    MODE{"Processing mode"}

    PI_TRACK["Pi RGB-D tracking<br/>and 3D mapping"]
    WEB["Web portal / DroneView<br/>3D map and camera feeds"]

    STREAM["Sensor stream over Wi-Fi"]
    PHONE["iPhone/iPad<br/>ARKit + rig alignment"]
    THERMAL["Thermal 3D reconstruction"]
    AR["Thermal AR display"]

    SENSORS --> PI
    PI --> MODE
    MODE -->|"Pi mode (default)"| PI_TRACK
    PI_TRACK --> WEB
    MODE -->|"Phone mode"| STREAM
    STREAM --> PHONE
    PHONE --> THERMAL
    THERMAL --> AR
    PHONE -. "pose and diagnostics" .-> PI

    classDef sensor fill:#e8f3ff,stroke:#2878b5,color:#102a43;
    classDef decision fill:#fff3cd,stroke:#b7791f,color:#5f370e;
    classDef output fill:#e6ffed,stroke:#27864c,color:#123d24;
    class SENSORS sensor;
    class MODE decision;
    class WEB,AR output;
```

The Pi always captures and synchronizes the drone sensors. In Pi mode it also
tracks and builds the RGB 3D map. In phone mode it streams the sensor data to a
LiDAR iPhone or iPad, where the thermal AR reconstruction is produced.
