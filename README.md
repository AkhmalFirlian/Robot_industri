# Tugas Midterm / UTS - ROS 2 Maze Runner

Repositori ini berisi implementasi simulasi robot menggunakan ROS 2 dan Gazebo untuk menyelesaikan tantangan navigasi labirin (*maze runner*).

---

## 👥 Anggota Kelompok

* **NIM:** 080
* **NIM:** 068

---

## 📋 Prasyarat System

Pastikan lingkungan Anda telah terkonfigurasi dengan:
* **ROS 2** (Humble / Iron / Rolling / Jazzy)
* **Gazebo / Ignition Gazebo**
* Package `ros_gz_bridge`
* Workspace ROS 2 berada pada folder `~/ros2_ws`

---

## 🚀 Cara Menjalankan (How to Run)

Untuk menjalankan simulasi dan skrip navigasi, buka 3 terminal terpisah lalu ikuti langkah-langkah di bawah ini:

### 1. Terminal 1: Launch Simulasi Gazebo
Jalankan lingkungan simulasi robot di Gazebo:
```bash
cd ~/ros2_ws
ros2 launch robin_bringup my_robot_gazebo.launch.py
```

### 2. Terminal 2: Bridge ROS-Gazebo (TF Topic)
Jalankan `ros_gz_bridge` untuk menjembatani topik pose/TF dari Gazebo ke ROS 2:
```bash
cd ~/ros2_ws
ros2 run ros_gz_bridge parameter_bridge '/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' --ros-args -r /world/empty/dynamic_pose/info:=/world_pose
```

### 3. Terminal 3: Jalankan Maze Runner
Jalankan skrip utama kontroler robot untuk menyelesaikan labirin:
```bash
cd ~/ros2_ws
python3 maze_runner.py
```

---

## 📁 Struktur Workspace (Ringkasan)

```text
~/ros2_ws/
├── src/
│   ├── robin_bringup/    # Launch files & paket konfigurasi Gazebo
│   └── ...
├── maze_runner.py        # Skrip utama navigasi robot
└── README.md
```
