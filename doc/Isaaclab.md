<h2 align="center">Part 1 : Train in Isaaclab</h2>

This version of **Isaac Lab** is a custom fork of NVIDIA Isaac Lab, including several improvements and modifications for the **Go2 Sim-to-Real pipeline** 🐾. It installs **exactly like the official Isaac Lab**, only the **clone URL** changes. Main modifications are in the definitions of rewards and their weights, and a foot contact sensor has been added to detect when each foot is in contact with the ground.


---
## ⚙️ System Requirements for this section

|  Component |  Recommended Version |
|--------------|------------------------|
| Nvidia Graphic cards | |
| **Ubuntu** | 22.04 LTS |
| **Python** | 3.10+ |



---

##  1️⃣ Create the Python Environment

Creating a dedicated Python environment is strongly recommended to avoid dependency conflicts and keep your setup reproducible.

### Using Conda (Recommended)
For **Isaac Sim 4.x** (Python 3.10):

```bash
conda create -n env_isaaclab python=3.10.18
conda activate env_isaaclab
```

For **Isaac Sim 5.x** (Python 3.11):

```bash
conda create -n env_isaaclab python=3.11
conda activate env_isaaclab
```


---

##  2️⃣ Clone Theo Bounac’s Isaac Lab Fork

Install Isaaclab with this fork : [Isaaclab_deploy](https://github.com/TheoBounac/Isaaclab_deploy)


---
## 🦾 3️⃣ Train 

Start training an RL agent directly inside Isaac Lab:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Unitree-Go2-v0 --num_envs=40
```

You can go headless mode with `--headless`.
This uses our custom Go2 configuration and environment.

---


## 🦾 ️4️⃣ Play

You can try your trained RL agent directly inside Isaac Lab:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task=Isaac-Velocity-Rough-Unitree-Go2-v0 --num_envs=10
```

---

<h3 align="center">✨ Enjoy training and deploying your Go2 robot with Isaac Lab! ✨</h3>
