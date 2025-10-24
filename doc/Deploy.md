# <h2 align="center">DEPLOY</h2>


**This section aims to **deploy** on the Go2 robot (Unitrre), a low level RL model (policy)**

<p align="center">
  <b>Simulation IsaacLab</b>
  <img src="isaaclab.gif" width="700">
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
      <img src="robot2.gif" style="width:100%; display:block; margin:auto;">
    </td>
    <td style="width:50%; text-align:center;">
      <img src="rviz2.gif" style="width:100%; display:block; margin:auto;">
    </td>
  </tr>
</table>


## 📁 Architecture of this section

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
└── README.md                 
```

---
## ⚙️ System Requirements for this section

|  Component |  Recommended Version |
|--------------|------------------------|
| **Ubuntu** | 22.04 LTS |
| **Python** | 3.10+ |
| **ROS 2** | Humble |
| **Isaac Sim / Isaac Lab** | 4.0.0+ |
| **CUDA (optionnel)** | 11.8+ |


---
<h2 align="center">🔧 Installation Guide🔧</h2> 

###  1️⃣ Env conda setup
Create a conda environment for the project :
```bash
conda create -n env_isaaclab python=3.10.18
conda activate env_isaaclab
```

---
Ensure the latest pip version is installed. To update pip, run the following command from inside the virtual environment :
```bash
pip install --upgrade pip
```

---
Install a CUDA-enabled PyTorch 2.7.0 build for CUDA 12.8 :
```bash
pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```


---
###  2️⃣ Clone the project

```bash
git clone https://github.com/TheoBounac/Deploy_SimToReal_Go2.git
cd Deploy_SimToReal_Go2
```
---
Install depedencies :
```bash
pip install -r requirements.txt
```

---
###  3️⃣ Clone SDK Unitree
unitree_sdk2py is a library used for communication with **Unitree** robots in python. 

Clone the repository using Git :
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
```
Environment Variable :
```bash
echo 'export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6' >> ~/.bashrc
```
Navigate to the directory and install it:
```bash
cd unitree_sdk2_python
pip install -e .
```

---

<h2 align="center">🚀 Run the project 🚀</h2> 

Once the installation is complete, follow these steps to launch an RL model on the Go2 robot.

1. Activate conda env :
   ```bash
   conda activate env_isaaclab
   ```
2. Navigate to `deploy_real`:
   ```bash
   cd ~/Deploy_SimToReal_Go2/deploy_real
   ```
3. Run `deploy_real_isaaclab.py`:
   ```bash
   python deploy_real_isaaclab.py enp0s31f6 go2.yaml
   ```

---


 7️⃣ 8️⃣ 9️⃣ 🔟




