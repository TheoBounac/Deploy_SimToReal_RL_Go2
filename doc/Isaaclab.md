<h2 align="center">🚀 Part 1 : Train in Isaaclab</h2>

This version of **Isaac Lab** is a custom fork of NVIDIA Isaac Lab,  
including several improvements and modifications for the **Go2 Sim-to-Real pipeline** 🐾.  
It installs **exactly like the official Isaac Lab**, only the **clone URL** changes.

---

## 🧩 1️⃣ Create the Python Environment

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

Update pip:
```bash
pip install --upgrade pip
```

---

## 💻 2️⃣ Install Isaac Sim (pip package)

Isaac Sim can be installed directly via pip (requires **GLIBC 2.35+**).  
If you encounter compatibility issues on Ubuntu 20.04 (GLIBC 2.31), use the binary version instead.

```bash
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

Install PyTorch (adapt the CUDA version to your GPU):
```bash
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

---

## 🧠 3️⃣ Clone Theo Bounac’s Isaac Lab Fork

Replace the NVIDIA repository link with the custom fork URL 👇

### HTTPS
```bash
git clone https://github.com/TheoBounac/IsaacLab.git
cd IsaacLab
```

*(optional, for developers with SSH keys)*  
```bash
git clone git@github.com:TheoBounac/IsaacLab.git
cd IsaacLab
```

---

## ⚙️ 4️⃣ Install Isaac Lab Extensions

On Linux:
```bash
sudo apt install cmake build-essential
./isaaclab.sh --install    # or  ./isaaclab.sh -i
```

You can also install only a specific framework (e.g. RSL-RL):
```bash
./isaaclab.sh --install rsl_rl
```

Available options:
```
all, rl_games, rsl_rl, sb3, skrl, robomimic, none
```

---

## 🧪 5️⃣ Verify the Installation

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

✅ You should see a **black simulation window** — if yes, the installation worked! 🎉  
Exit with `Ctrl + C` in your terminal.

---

## 🦾 6️⃣ Train a Robot (Example)

Start training an RL agent directly inside Isaac Lab:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Go2-v0 --headless
```

This uses Theo Bounac’s custom Go2 configuration and environment.

---

## 🧱 7️⃣ Notes for Users of This Fork

🔹 This fork includes custom files for **Go2 locomotion and Sim2Real adaptation**:
- Modified task configs → `flat_env_cfg.py`, `rough_env_cfg.py`
- Enhanced reward functions → `rewards.py`
- Updated RL scripts → `train.py`, `play.py`
- New PPO agent config → `sb3_rough_ppo_cfg.yaml`

🔹 All other features, tools, and commands remain identical to the **official Isaac Lab**.

---

## 🔄 8️⃣ Keeping Your Fork Up to Date (Optional)

If you want to sync future updates from NVIDIA’s official Isaac Lab:

```bash
# Add the original repository as upstream
git remote add upstream https://github.com/NVIDIA-Omniverse/IsaacLab.git

# Fetch new commits
git fetch upstream

# Merge them into your fork
git merge upstream/main
```

This way, your fork stays aligned with new Isaac Lab releases  
while preserving your Go2-specific improvements.

---

## 💡 Summary

Only **one line** differs from the official installation:

```diff
- git clone https://github.com/NVIDIA-Omniverse/IsaacLab.git
+ git clone https://github.com/TheoBounac/IsaacLab.git
```

Everything else — environment setup, simulator installation, and workflows — remains **exactly the same** ✅

---

<h3 align="center">✨ Enjoy training and deploying your Go2 robot with Isaac Lab! ✨</h3>
<p align="center">
  <img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExZDl0aHk0eTBrM2wwd3pqenQ1b3F1c3J5eDlxaHd0YjFwaDUxMG03YiZlcD12MV9naWZzX3NlYXJjaCZjdD1n/MFQ8TnxgU3JZm/giphy.gif" width="400">
</p>
