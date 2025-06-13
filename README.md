# 🛸 Multi-Drone PX4 SITL Simulation with Realsense Support

This package enables **multi-drone Software-in-the-Loop (SITL)** simulations using PX4 and Gazebo Classic with:

* ✅ Support for **multiple IRIS quadrotors**
* 📸 Optional integration of **Intel Realsense D435i** (simulated)
* 🌍 A **supervisor node** for managing transformations and common coordinate frames

> **🧪 If it works here, it works in the real world.**
> This simulation is designed to closely mimic real flight conditions, especially for multi-agent coordination and localization setups.

## 🚀 Launch Files

This package provides a set of ROS launch files to start different simulation configurations:

| Launch File        | Description                                                                                                                          |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------ |
| `iris_sim.launch`  | Launches a multi-drone simulation **without cameras**. Ideal for performance testing or formation flight.                            |
| `uvify_sim.launch` | Launches a multi-drone simulation **with Realsense D435i** sensors for each drone. Use for perception, SLAM, and vision-based tasks. |

More launch files will be added soon.

## ➕ Spawning Multiple Drones

To simulate multiple drones:

1. Open your desired launch file (e.g., `uvify_sim.launch`).
2. Locate the following XML group:

   ```xml
   <group ns="uav0">
     ...
   </group>
   ```
3. **Copy and paste** the group block for each additional drone.
4. Update:

   * The `ns` attribute (`uav1`, `uav2`, etc.)
   * All `ID`, `namespace`, or related identifiers inside the block.

> [!CAUTION]
> **Ensure unique starting positions** for each drone to avoid collisions in simulation!
> Update the initial `<x>`, `<y>`, `<z>` coordinates for every robot instance.

## 🧭 The Supervisor Node

The **supervisor** node plays a critical role in multi-drone coordination:

* Establishes a **common world frame** across all drones.
* Manages frame transforms:

  ```
  map → uavX/local_origin → uavX/base_link
  ```
* Publishes **relative odometry** for each UAV with respect to the global fixed frame.

> \[!NOTE]
> In real-world deployments, this is analogous to the **shared GPS frame** every drone uses. In simulation, we mimic this using fixed transforms and simulated position feedback.

## 🌐 Geodetic Reference & Local Origin

The supervisor defines each drone’s `local_origin` using **geodetic coordinates** (latitude/longitude). This is converted into the simulation frame using the [`pyproj`](https://pypi.org/project/pyproj/) library.

### 📦 Install `pyproj`:

```bash
pip install pyproj
```

> [!WARNING]
> Accurate GPS data or a high-quality EKF2 configuration is **essential** for real-world applications.
> Poor position estimates can lead to large transform drift and localization errors.

## 🛠 Configuration

To configure which drones are spawned and tracked:

1. Open the file:

   ```
   config/config.yaml
   ```
2. Set the list of UAV IDs you want to include in the simulation or real-world setup. For example:

   ```yaml
   drone_ids: [0, 1, 2]
   ```

> [!TIP]
> These IDs should **match** the namespaces used in your launch file (`uav0`, `uav1`, ...).

## 🔧 Coming Soon

We're actively improving this package. Upcoming features include:

* 🔄 Dynamic UAV spawner
* 📡 MAVLink telemetry multiplexing
* 🗺️ Map server integration
* 🎥 Enhanced Realsense simulation realism

## 🤝 Contributing

Have ideas or found a bug? Contributions are welcome!

* Open an [issue](https://github.com/your-repo/issues)
* Submit a [pull request](https://github.com/your-repo/pulls)
