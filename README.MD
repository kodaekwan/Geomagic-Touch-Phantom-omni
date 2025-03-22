# Phantom Omni Standalone Interface (No ROS)

This project enables standalone control of the Geomagic Touch / Phantom Omni device **without ROS**.  
It provides a C++ backend and a Python frontend using shared memory for communication.

---

## 🖥️ Test Environment

### [OS]
- Windows 11 + Ubuntu 20.04 (WSL2)

### [WSL2 Settings]
`.wslconfig`:
```ini
networkingMode=mirrored
```

---

## ⚙️ Setup Instructions

### OpenHaptics SDK
- SDK Download: [OpenHaptics for Linux Developer Edition v3.4](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US)

### Geomagic Touch Driver
- Driver file: `geomagic_touch_device_driver_2016.1-1-amd64.zip`

---

## 🧱 Install Dependencies

### C++ Dependencies

```bash
sudo apt update
sudo apt install g++
sudo apt install freeglut3-dev
sudo apt install libncurses5-dev
sudo apt install zlib1g-dev
sudo apt install libncurses5
```

### Python Dependencies

```bash
pip install sysv-ipc
pip install numpy
```

---

## 🛠️ Build (CMake)

```bash
mkdir your_dir
cd your_dir
git clone <REPO_URL>
cd your_dir
cmake .
make
```

---

## ▶️ Run

### C++ Side

```bash
./build/omni shm=777 hz=1000
```

### Python Side

```bash
python3 omni_demo.py
```

---

## 📚 References

This project is based on:

- [phantom_omni by danepowell](https://github.com/danepowell/phantom_omni) — licensed under the MIT License

### ✍️ Modifications by DaeKwan (2024)
- Removed ROS dependency
- Integrated shared memory access
- Refactored for standalone C++ use
- Applied consistent time handling and real-time filtering
- Organized build structure and simplified runtime parameters

---

## © License

This project is distributed under the MIT License.  
See the [LICENSE](./LICENSE) file for more information.