# 🛸 Autonomous Drone Red Object Tracking & Search - Webots

Ce projet présente la simulation d'un drone quadricoptère autonome (DJI Mavic 2 Pro) sous **Webots**, capable de scanner un environnement, de détecter un véhicule/objet rouge via traitement d'image, et de s'y asservir en vol stationnaire.

---

## 📷 Aperçu de la simulation

![Aperçu de la simulation](Images/apercu.png)

---

## ⚙️ Architecture & Machine à États (FSM)

Le contrôleur Python (`Scanning.py`) implémente une machine à états finis pour la recherche et l'approche :

1. **`SCANNING` :** Le drone pivote sur son axe de lacet (`YAW`) pour balayer l'environnement.
2. **`APPROACHING` :** Déclenché dès que l'objet rouge dépasse le seuil de détection (`ratio > 0.02`). Le drone s'oriente et avance vers la cible (`pitch = -1.0`).
3. **`HOVERING` :** Déclenché lorsque le drone est à proximité de la cible (`ratio > 0.25`). Il se stabilise en vol stationnaire directement au-dessus de l'objet.

> **Mode Manuel :** Une interruption via le clavier (flèches directionnelles + SHIFT) permet d'intervenir manuellement avant de repasser en contrôle automatique.

---

## 🛠️ Spécifications techniques

- **Simulateur :** Webots (Drone DJI Mavic 2 Pro)
- **Langage & Librairies :** Python, NumPy, API Webots (`controller`)
- **Asservissement :** Correcteurs P pour le maintien d'altitude (GPS) et le contrôle d'attitude Roll/Pitch (IMU & Gyroscope)
- **Traitement d'image :** Caméra RGB avec segmentation par masquage de couleur et calcul du centroïde `(cx, cy)`

---

## 📂 Organisation du dépôt

- **`/controllers/Scanning/`** : Code source du contrôleur Python (`Scanning.py`).
- **`/worlds/`** : Scène de simulation Webots (`drone_red_object_search.wbt`).
- **`/Images/`** : Captures d'écran et rendus de la simulation.
