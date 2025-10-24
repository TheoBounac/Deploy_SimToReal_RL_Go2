# <h2 align="center">DEPLOY SIM-TO-REAL RL MODEL ON Go2</h2>


**This repository aims to train a **Reinforcement Learning (RL)** model on the **Unitree Go2 quadruped robot** in simulation (IsaacLab) and deploy it on the real robot.
It also includes the use of a **Kalman filter** for pose and velocity estimation, ensuring accurate state feedback during real-world deployment.**


<p align="center">
  <b>Simulation IsaacLab</b>
  <img src="doc/isaaclab.gif" width="700">
  <br>
</p>






<table align="center" style="border-collapse:collapse;">
<th style="width:50%; text-align:center;">
  <div style="display:inline-block; width:200px;">Deploy on real robot </div>
</th>
<th style="width:50%; text-align:center;">
  <div style="display:inline-block; width:200px;">Real time Kalman filter</div>
</th>

  <tr>
    <td style="width:50%; text-align:center;">
      <img src="doc/robot2.gif" style="width:100%; display:block; margin:auto;">
    </td>
    <td style="width:50%; text-align:center;">
      <img src="doc/rviz2.gif" style="width:100%; display:block; margin:auto;">
    </td>
  </tr>
</table>




---
## Project overview

This project implements a complete **Sim-to-Real** pipeline:

 - 🎮 How to train Reinforcement Learning (RL) policies on **IsaacLab Simulation**

 - 🤖 How to **deploy models on the real Go2 robot** via the Unitree SDK

 - 🔄 How to use **ROS 2 Communication** and **Kalman filter (Inria Paris)** for real-time control and sensor/command integration

The project combines **Python + ROS 2 + IsaacLab + Kalman filter (Inria Paris)**, enabling training, testing, and transferring an RL policy to the real robot.

---
## 📁 Architecture

```
deploy_go2/
│
├── deploy_real/                # Deployment scripts for Go2
│   ├── config.py
│   ├── deploy_real_isaaclab.py
│   └── node_kalman.py
│
├── pre_train/                  # Pre-trained RL models (policies)
│   ├── policy_rough.pt
│   └── ...
│
├── unitree_sdk2_python/        # SDK Unitree
│
├── go2_ws/                     # Kalman Filter for Go2 (Inria Paris)
│
└── README.md                 

Isaaclab
```

---
## ⚙️ System Requirements

|  Component |  Recommended Version |
|--------------|------------------------|
| **Ubuntu** | 22.04 LTS |
| **Python** | 3.10+ |
| **ROS 2** | Humble |
| **Isaac Sim / Isaac Lab** | 4.0.0+ |
| **CUDA (optionnel)** | 11.8+ |


---
<h2 align="center">🔧 Installation Guide🔧</h2> 

[📘 Detailed Installation Guide](doc/Deploy.md)

---

##  Links

| 🔗 Resources | 📍 Link |
|--------------|---------|
|  **IsaacLab (NVIDIA)** | [https://github.com/isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) |
|  **Unitree SDK2 Python** | [https://github.com/unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) |
|  **Projet principal** | [https://github.com/TheoBounac/Deploy_SimToReal_Go2](https://github.com/TheoBounac/Deploy_SimToReal_Go2) |


---


##  Author

**Théo Bounaceur**  
Laboratory **LORIA** (**CNRS** / **University of Lorraine**), Nancy in France  
🧬 Développement : IsaacLab · ROS 2 · Unitree SDK2  
📫 Contact : theo.bounaceur@loria.fr

---

---
###  4️⃣ Cloner go2_odometry
```bash
cd ..
git clone https://github.com/inria-paris-robotics-lab/go2_odometry.git
```

---
###  5️⃣ Installer Isaaclab
Cette partie est optionnelle, elle permet d'entraîner sois-même des modèles de RL. Des modèles pré-entraînés sont déja disponibles dans `pre_train`. 

Pour installer Isaaclab, vous pouvez vous référer au [guide isaaclab](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html).
Les commandes importantes sont rappelées ci-dessous.

Install the Isaac Lab packages along with Isaac Sim :
```bash
pip install isaaclab[isaacsim,all]==2.2.0 --extra-index-url https://pypi.nvidia.com
```
Make sure that your virtual environment is activated. Check that the simulator runs as expected :
```bash
isaacsim
```

The first run will prompt users to accept the Nvidia Omniverse License Agreement. To accept the EULA, reply `Yes` when prompted with the below message:
```bash
By installing or using Isaac Sim, I agree to the terms of NVIDIA OMNIVERSE LICENSE AGREEMENT (EULA)
in https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html

Do you accept the EULA ? (Yes/No): Yes
```
Cela devrait prendre quelque minutes pour la première run.

Lancer un entrainement du go2 :
```bash
cd isaaclab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py  --task Isaac-Velocity-Rough-Unitree-Go2-v0  --num_envs 4080  --max_iterations 9999 --headless
```

Tester le modèle :
```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py  --task Isaac-Velocity-Rough-Unitree-Go2-v0  --num_envs 4
```
 7️⃣ 8️⃣ 9️⃣ 🔟




