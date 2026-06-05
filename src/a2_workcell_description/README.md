# A2 Workcell Description

Dit ROS 2-pakket bevat de URDF/Xacro-beschrijving van de werkomgeving voor de UFACTORY Lite 6-robot.

De omgeving bestaat voorlopig uit:

* een tafelblad;
* vier tafelpoten;
* een verhoogd werkplatform;
* steunpoten onder het werkplatform;
* een robotmontageplaat;
* een stoel;
* een montageframe voor de Lite 6.

## Vereisten

Dit pakket is ontwikkeld voor:

* Ubuntu 24.04;
* ROS 2 Jazzy;
* RViz2;
* Xacro;
* `robot_state_publisher`.

Installeer de benodigde pakketten met:

```bash
sudo apt update
sudo apt install ros-jazzy-xacro liburdfdom-tools
```

## Workspace bouwen

Ga naar de hoofdmap van de ROS 2-workspace:

```bash
cd ~/Git-projects/AI_driven_robotics_project_A2
```

Bouw alleen het workcell-pakket:

```bash
colcon build \
  --symlink-install \
  --packages-select a2_workcell_description
```

Source daarna de workspace:

```bash
source install/setup.bash
```

## Xacro-bestand controleren

Ga naar het pakket:

```bash
cd ~/Git-projects/AI_driven_robotics_project_A2/src/a2_workcell_description
```

Zet het Xacro-bestand tijdelijk om naar URDF:

```bash
xacro urdf/workcell.urdf.xacro > /tmp/workcell.urdf
```

Controleer daarna of de URDF geldig is:

```bash
check_urdf /tmp/workcell.urdf
```

Bij een correcte configuratie verschijnt onder andere:

```text
Successfully Parsed XML
```

## Omgeving starten in RViz

Ga terug naar de workspace:

```bash
cd ~/Git-projects/AI_driven_robotics_project_A2
```

Source de workspace:

```bash
source install/setup.bash
```

Start daarna de launchfile:

```bash
ros2 launch a2_workcell_description display_workcell.launch.py
```

## RViz instellen

Wanneer RViz voor de eerste keer opent:

1. Stel bij **Global Options** de `Fixed Frame` in op:

```text
environment_root
```

2. Klik linksonder op **Add**.

3. Voeg een `RobotModel` toe.

4. Stel bij `Description Topic` het volgende topic in:

```text
/robot_description
```

De werkomgeving hoort daarna zichtbaar te worden.

## Belangrijke bestanden

```text
a2_workcell_description/
├── launch/
│   └── display_workcell.launch.py
├── rviz/
├── urdf/
│   └── workcell.urdf.xacro
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Na wijzigingen opnieuw bouwen

Na wijzigingen aan de launchfile, `CMakeLists.txt`, `package.xml` of andere geïnstalleerde bestanden:

```bash
cd ~/Git-projects/AI_driven_robotics_project_A2

colcon build \
  --symlink-install \
  --packages-select a2_workcell_description

source install/setup.bash
```

Start daarna opnieuw:

```bash
ros2 launch a2_workcell_description display_workcell.launch.py
```

## Problemen oplossen

### De foutmelding `Frame [map] does not exist`

Verander in RViz de `Fixed Frame` van:

```text
map
```

naar:

```text
environment_root
```

### De omgeving is niet zichtbaar

Controleer of `RobotModel` aan RViz is toegevoegd en of het description topic staat op:

```text
/robot_description
```

Controleer daarnaast of de Xacro geldig is:

```bash
xacro urdf/workcell.urdf.xacro > /tmp/workcell.urdf
check_urdf /tmp/workcell.urdf
```

### Het pakket wordt niet gevonden

Source de workspace opnieuw:

```bash
source ~/Git-projects/AI_driven_robotics_project_A2/install/setup.bash
```

Controleer vervolgens:

```bash
ros2 pkg list | grep a2_workcell_description
```
