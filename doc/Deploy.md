# <h2 align="center">DEPLOY</h2>


**This section aims to **deploy** on the Go2 robot, a low level RL model (policy). You can either train an RL model in IsaacLab (see **Part 1 :** [How to train RL policies in IsaacLab Simulation](https://github.com/TheoBounac/Deploy_SimToReal_RL_Go2/blob/main/doc/Isaaclab.md)) or use a pre-trained model provided in this project.**


<p align="center">
  <img src="robot2.gif" width="700">
  <br>
</p>




## 📁 Architecture of this section

```
Deploy_SimToReal_Go2/
  ├── deploy_real/                # Deployment scripts for Go2
  │   ├── config.py
  │   ├── deploy_real_isaaclab.py
  │   └── node_kalman.py
  │
  ├── pre_train/                  # Pre-trained RL models (policies)
  │   ├── policy_rough.pt
  │   └── policy_rough_2.pt
  │
  ├── unitree_sdk2_python/        # SDK Unitree
  │
  └── README.md

kalman_filter/                    # Kalman Filter for Go2 (Inria Paris)          
```

---
## ⚙️ System Requirements for this section

|  Component |  Recommended Version |
|--------------|------------------------|
| Go2 robot (Unitree) | Edu with feet sensors|
| **Ubuntu** | 22.04 LTS |
| **Python** | 3.10+ |
| **ROS 2** | Humble |



---
<h2 align="center">🔧 Installation Guide : Deploy🔧</h2> 

###  1️⃣ Env conda setup
Create a conda environment for the project :
```bash
conda create -n env_isaaclab python=3.10.18
conda activate env_isaaclab
```


Ensure the latest pip version is installed. To update pip, run the following command from inside the virtual environment :
```bash
pip install --upgrade pip
```


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

Install depedencies :
```bash
pip install -r requirements.txt
```

---
###  3️⃣ Clone SDK Unitree
`Unitree_sdk2py` is a Python library that enables direct communication with Unitree robots.
It plays a crucial role in this project, allowing the system to collect sensor data from the robot and send velocity and motor commands in real time.

Clone the repository using Git :
```bash
cd ~/Deploy_SimToReal_Go2
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
```

Navigate to the directory and install it:
```bash
cd unitree_sdk2_python
pip install -e .
```

At this point, your folder should look like this :

 <p align="center">
  <img src="plan.png" width="700">
  <br>
 </p>
 
---
###  4️⃣ Clone Go2_odometry
Install the Kalman filter for the Go2 : [Github Inria Paris Go2_odometry](https://github.com/inria-paris-robotics-lab/go2_odometry?tab=readme-ov-file). It provides an estimation of the robot’s velocity, which is one of the input features of the neural network used for control.

Here is a full tutorial on how to install this library :

[📘 How to use  **Kalman filter (Inria Paris)** for real-time control and sensor/command integration](Deploy_with_Kalman_filter.md)

---

<h2 align="center">🚀 Run the project : Deploy 🚀</h2> 

Once the installation is complete, follow these steps to launch an RL model on the Go2 robot.

1. Activate conda env :
   ```bash
   conda activate env_isaaclab
   ```
2. Connect the robot with ethernet
   
   Turn on the robot and connect it to your PC using an Ethernet cable.
   Go to Settings/Network and then IPv4. Fill the gaps as follows :
   
   <p align="center">
    <img src="ipv4.png" width="700">
    <br>
   </p>
3. Collect the network interface

   Type in the terminal, and collect the adress :

   ```bash
   ifconfig
   ```
   <p align="center">
    <img src="net.png" width="700">
    <br>
   </p>

---
⚠️ **Safety Notice:**  
From now on, make sure the robot is standing in an open area, free of obstacles or people nearby.

---

4. Activate the `kalman_filter` (be sure to complete [📘 How to use  **Kalman filter (Inria Paris)** for real-time control and sensor/command integration](doc/Deploy_with_Kalman_filter.md) before) :
   
   Open a new terminal
   
   Activate env conda :
   ```bash
   conda activate go2_odometry_env
   ```
   Source :

   ```bash
   # In Conda: keep this to avoid GLIBCXX mismatches with rclpy
   export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
  
   source /opt/ros/humble/setup.bash         # Make sure it is sourced
   source ~/kalman_filter/install/setup.bash #
   source /opt/ros/humble/setup.bash         #
   ```
   
   And launch with :

   ```bash
   ros2 launch go2_odometry go2_odometry_switch.launch.py odom_type:=use_full_odom
   ```




5. Run deploy script  :

   Open a new terminal
   
   Activate env conda :
   ```bash
   conda activate env_isaaclab
   ```
   
    Navigate to `deploy_real`:
    
     ```bash
     cd ~/Deploy_SimToReal_Go2/deploy_real
     ```
  
     Run `deploy_real_isaaclab.py `with your network interface :
     
     ```bash
     export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
     source ~/kalman_filter/install/setup.bash
     python deploy_real_isaaclab.py enp0s31f6 go2.yaml
     ```

     You should have someting like this :
     <p align="center">
      <img src="sisi.png" width="800">
      <br>
    </p>

6. Visualize on Rviz2
       Open a new terminal and launch rviz2 with :
    
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/go2_ws/install/setup.bash
    rviz2
    ```
    
    To add the robot, just click on `add` then `RobotModel` :
    <p align="center">
     <img src="so.png" width="600">
     <br>
    </p>
    
    And make sure to be on FixedFrame: `odom` and DescriptionTopic: `/robot_descrption` (provided by the kalman filter) :
    <p align="center">
     <img src="soso.png" width="300">
     <br>
    </p>
    
    
    You should normally see your robot moving in real time :
    
     <p align="center">
      <img src="preskalm.png" width="1100">
      <br>
     </p>

   ---
🦾 **Deployment Sequence**
1. Run the command to launch `deploy_real_isaaclab.py` with the `go2.yaml` configuration. This file contains important configurations values for the deployment with the Go2.
2. The robot will exit its default mode and lie down smoothly.
        <p align="center">
      <img src="robcouche.jpg" width="300">
      <br>
    </p>

3. Press the **`Start`** button on the remote to make the robot stand up.
         <p align="center">
      <img src="robdebout.jpg" width="300">
      <br>
    </p>
4. Press the **`A`** button to activate the trained model.
   
<div align="center">

---
⚠️ **Emergency Stop** ⚠️  

If the robot behaves unexpectedly or becomes dangerous, **press the `Select` button immediately**.  
The model will be stopped, and the robot will safely fold its legs.  

</div>





