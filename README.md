# <h2 align="center">DEPLOY SIM-TO-REAL RL MODEL ON Go2</h2>



>  **Ce repository vise à entraîner un modèle de Reinforcement Learning (RL) sur le robot quadrupède Unitree Go2** en simulation (IsaacLab), et à le déployer sur le robot réel.

---

## Aperçu général

Ce projet met en place une chaîne **Sim-to-Real** complète :
- 🎮 **Simulation IsaacLab** pour l’entraînement des politiques RL  
- 🤖 **Déploiement sur le robot Go2 réel** via le SDK Unitree  
- 🔄 **Communication ROS 2** pour le contrôle en temps réel et l’intégration capteurs/commandes  

Le projet combine **Python + ROS 2 + IsaacLab**, permettant d'entraîner, tester et transférer une politique RL vers le robot réel.

---

## 📁 Structure du projet
Voici une vue d’ensemble du projet et de son architecture finale :
```
Deploy_Model_Go2/
│
├── deploy_real/                # Scripts de déploiement sur le robot Go2 (réel/simulé)
│   ├── config.py
│   ├── deploy_real_isaaclab.py
│   └── node_kalman.py
│
├── pre_train/                  # Modèles RL pré-entraînés (policies)
│   ├── go2/
│   │   ├── policy_1.pt
│   │   └── ...
│
├── unitree_sdk2_python/        # SDK Unitree (à cloner séparément)
│
└── README.md                 

Isaaclab
```

---

## ⚙️ System Requirements

|  Composant |  Version recommandée |
|--------------|------------------------|
| **Ubuntu** | 22.04 LTS |
| **Python** | 3.10+ |
| **ROS 2** | Humble |
| **Isaac Sim / Isaac Lab** | 4.0.0+ |
| **CUDA (optionnel)** | 11.8+ |

---

<h2 align="center">🔧 Installation complète 🔧</h2> 


###  1️⃣ Setup

Crée un environnement conda pour le projet:

```bash
conda create -n env_isaaclab python=3.11
conda activate env_isaaclab
```

---

Ensure the latest pip version is installed. To update pip, run the following command from inside the virtual environment:

```bash
pip install --upgrade pip
```

---

Install a CUDA-enabled PyTorch 2.7.0 build for CUDA 12.8:
```bash
pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

---

###  1️⃣ Créer un workspace

Crée un répertoire principal qui contiendra tout le projet :

```bash
mkdir ~/Deploy_RL_Model_Go2
cd ~/Deploy_RL_Model_Go2
```

---

###  2️⃣ Cloner le projet principal

```bash
git clone https://github.com/TheoBounac/Deploy_SimToReal_Go2.git
cd Deploy_SimToReal_Go2
```

---

###  3️⃣ Cloner le SDK Unitree (à l’intérieur du projet)

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
```

---

###  4️⃣ Télécharger IsaacLab (à côté du projet)

```bash
cd ..
git clone https://github.com/isaac-sim/IsaacLab.git
```

🧭 **Attention :**
IsaacLab doit être placé **au même niveau** que `Deploy_SimToReal_Go2`,  
et **non à l’intérieur** :

```
~/Deploy_RL_Model_Go2/
├── Deploy_SimToReal_Go2/
└── IsaacLab/
```

---


<h2 align="center">🚀 Utilisation 🚀</h2> 

1. Active ton environnement Python :
   ```bash
   conda activate env_isaaclab
   ```
2. Lancer le script principal :
   ```bash
   python deploy_real/deploy_real_isaaclab.py
   ```
3. (Optionnel) Connecter ROS 2 :
   ```bash
   ros2 topic echo /odom
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...
   ```

---

##  Liens utiles

| 🔗 Ressource | 📍 Lien |
|--------------|---------|
|  **IsaacLab (NVIDIA)** | [https://github.com/isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) |
|  **Unitree SDK2 Python** | [https://github.com/unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) |
|  **Projet principal** | [https://github.com/TheoBounac/Deploy_SimToReal_Go2](https://github.com/TheoBounac/Deploy_SimToReal_Go2) |


---


##  Auteur

**Théo Bounaceur**  
Laboratoire **LORIA (CNRS / Université de Lorraine)**  
🧬 Développement : IsaacLab · ROS 2 · Unitree SDK2  
📫 Contact : theo.bounaceur@loria.fr

---

