# Dokumentácia projektu Runner

## 1. Účel projektu

Projekt je určený na simuláciu mobilných robotov pre diplomovú prácu zameranú na lokalizačné algoritmy. Simulácia používa ROS 2 Jazzy a Gazebo Sim 8.

Hlavným cieľom je vytvoriť prostredie, v ktorom sa môže pohybovať viac robotov naraz. Senzory jednotlivých robotov majú poskytovať iba merania dostupné lokalizačnému algoritmu. Skutočná poloha robota sa publikuje oddelene a používa sa iba na vyhodnotenie presnosti lokalizácie.

Robot sa nazýva **Runner**. Jednotlivé inštancie používajú názvy:

```text
runner_01
runner_02
...
runner_06
```

## 2. Fyzický model robota

Runner je diferenciálne riadený mobilný robot so styrmi poháňanými kolesami ale v simulacii je zatial pozita struktura dvoch pohananych a dvoch na stabilizaciu

| Parameter | Hodnota |
|---|---:|
| šírka tela | 0,6 m |
| dĺžka tela | 1,0 m |
| výška tela | 1,5 m |
| polomer kolesa, počiatočný odhad | 0,12 m |
| vzdialenosť kolies, počiatočný odhad | 0,52 m |
| maximálna lineárna rýchlosť | 2,0 m/s |
| maximálna uhlová rýchlosť | pi/2 rad/s |

Telo robota je modelované ako plný kváder. Jeho kolízna geometria je zámerne zachovaná, aby robot blokoval lúče LiDARu ostatných robotov. Vďaka tomu môže simulácia reprezentovať situácie, v ktorých si roboti navzájom prekážajú vo vnímaní okolitého priestoru.

Presné hodnoty hmotnosti, momentov zotrvačnosti, rozmerov kolies a trenia sú zatiaľ počiatočné odhady. Po získaní meraní skutočného robota sa musia nahradiť nameranými hodnotami.

## 3. Senzory

### 3.1 LiDAR

Runner má dva 2D LiDARy:

- predný pravý LiDAR (`front_right_lidar`),
- zadný ľavý LiDAR (`rear_left_lidar`).

Oba LiDARy sú umiestnené vo výške 1,6 m, teda mierne nad hornou plochou
robotického kvádra. Ich pôdorysné súradnice sú mimo rozmerov kvádra, aby
kolízna geometria vlastného robota nezakrývala celé okolie. Predný LiDAR je
orientovaný dopredu a zadný LiDAR dozadu. Počiatočná konfigurácia používa
rozsah 30 m, minimálnu vzdialenosť 0,12 m, frekvenciu 10 Hz a uhlové pokrytie
270 stupňov.

Kryty LiDARov nemajú vlastnú kolíznu geometriu. Robotické telo však kolíznu geometriu má a má byť v meraní viditeľné. V budúcej konfigurácii sa preto musí vyriešiť maskovanie vlastného tela v prijatom skene. LiDAR zároveň nesmie používať ground truth ani inú informáciu o skutočnej polohe robota.

### 3.2 IMU

IMU je umiestnená v samostatnom referenčnom rámci `imu_link`. Počiatočná frekvencia merania je 100 Hz. Štandardná odchýlka šumu je zatiaľ nastavená na:

- uhlová rýchlosť: 0,01,
- lineárne zrýchlenie: 0,02.

Tieto hodnoty sú dočasné a majú byť nahradené charakteristikou reálneho senzora.

### 3.3 Odometia a enkódery

Plugin diferenciálneho pohonu publikuje odometriu kolies. Táto odometria predstavuje merací vstup lokalizácie, nie ground truth. V ďalšej fáze sa musí overiť, či simulované chyby odometrie dostatočne reprezentujú chyby reálneho robota, najmä:

- nepresnosť polomeru kolies,
- nepresnosť rozchodu kolies,
- preklz,
- rozdielne opotrebovanie ľavého a pravého kolesa,
- diskretizáciu merania enkóderov.

## 4. Oddelenie meraní a ground truth

Ground truth je publikovaný samostatne ako odometria na téme:

```text
/<meno_robota>/ground_truth/odometry
```

Napríklad:

```text
/runner_01/ground_truth/odometry
```

Lokalizačný algoritmus túto tému nesmie odoberať. Ground truth sa používa iba v hodnotiacej časti experimentu na výpočet chyby odhadu.

Vhodné metriky vyhodnotenia sú:

- euklidovská chyba polohy:

  ```text
  e_p = sqrt((x_odhad - x_skutocnost)^2 + (y_odhad - y_skutocnost)^2)
  ```

- chyba orientácie po normalizácii uhla do intervalu `[-pi, pi]`,
- RMSE polohy,
- MAE polohy,
- maximálna chyba,
- čas potrebný na konvergenciu lokalizácie,
- prípadne robustnosť pri výpadkoch alebo zhoršení meraní.

Pri zápise metodiky treba jasne uviesť, že ground truth slúži na spätné vyhodnotenie a nie je súčasťou vstupu lokalizačného algoritmu.

## 5. Štruktúra projektu

```text
Diploma/
├── DOKUMENTACIA_RUNNER_SIM.md
├── run_runner_sim.sh
└── diploma_ws/
    ├── src/
    │   └── runner_sim/
    │       ├── CMakeLists.txt
    │       ├── package.xml
    │       ├── README.md
    │       ├── config/
    │       │   └── runner_sim.yaml
    │       ├── launch/
    │       │   └── runner_sim.launch.py
    │       ├── models/
    │       │   └── runner.urdf.xacro
    │       └── worlds/
    │           ├── runner_world.sdf
    │           └── warehouse.sdf
    ├── build/
    ├── install/
    └── log/
```

### Význam súborov

- `models/runner.urdf.xacro` obsahuje robotický model, rámy, kolesá, LiDAR, IMU, diferenciálny pohon a Gazebo pluginy.
- `launch/runner_sim.launch.py` spúšťa Gazebo, robot state publisher, spawnovanie robotov a ROS-Gazebo bridge. Počet robotov je nastaviteľný od jednej do šiestich inštancií.
- `worlds/runner_world.sdf` obsahuje základné simulačné prostredie s fyzikou, osvetlením a podlahou.
- `worlds/warehouse.sdf` obsahuje opakovateľné skladové prostredie s obvodovými stenami a regálmi.
- `config/runner_lidar.rviz` obsahuje 2D RViz konfiguráciu pre oba LiDARy robota `runner_01`, v buducnosti v nom budu vsetky lidarove data
- `config/runner_sim.yaml` slúži ako centrálne miesto pre rozmery, limity a predbežné parametre senzorov.
- `run_runner_sim.sh` automatizuje zostavenie a spustenie simulácie a umožňuje jej ukončenie klávesom `q`.
- `package.xml` deklaruje závislosti ROS 2 balíka.
- `README.md` obsahuje stručný návod na zostavenie a spustenie.

## 6. Témy a rozhrania

Pre robota `runner_01` sú plánované najmä tieto témy:

| Téma | Typ | Účel |
|---|---|---|
| `/runner_01/cmd_vel` | `geometry_msgs/msg/Twist` | riadenie rýchlosti |
| `/runner_01_lidar_front` | `sensor_msgs/msg/LaserScan` | predný pravý LiDAR |
| `/runner_01_lidar_rear` | `sensor_msgs/msg/LaserScan` | zadný ľavý LiDAR |
| `/runner_01/lidar_fused` | `sensor_msgs/msg/LaserScan` | fúzia predného a zadného LiDARu v rámci `base_link` |
| `/runner_01/sensors/imu/data` | `sensor_msgs/msg/Imu` | meranie IMU |
| `/runner_01/wheel/odometry` | `nav_msgs/msg/Odometry` | odometria kolies |
| `/runner_01/ground_truth/odometry` | `nav_msgs/msg/Odometry` | vyhodnocovacia skutočná poloha a rýchlosť |

Pri šiestich robotoch sa rovnaké rozhranie opakuje pod príslušným prefixom `runner_01` až `runner_06`. Namespacing je dôležitý, aby sa správy robotov nemiešali.

Predný a zadný LiDAR sa vo lokalizačnej node najprv transformujú do stredu
robotického rámu `base_link`. Použité 2D transformácie vychádzajú z geometrie
modelu:

```text
front: p_base = R(0) p_front + [ 0.56, -0.36 ]^T
rear:  p_base = R(pi) p_rear  + [-0.56,  0.36 ]^T
```

Body sa následne rasterizujú do jedného 360-stupňového `LaserScan` s rámom
`/<robot_name>/base_link` a publikujú sa na `/<robot_name>/lidar_fused`.
Tento zlúčený sken je vstupom ICP a EKF. Pôvodné topics predného a zadného
LiDARu zostávajú zachované na samostatnú kontrolu a diagnostiku.

## 7. Matematický model ICP a EKF lokalizácie

Lokalizačný algoritmus je implementovaný v samostatnej C++ node
`lidar_localization_node`. Algoritmus odhaduje rovinnú polohu mobilného robota
v stavovom vektore

$$
\mathbf{x}_k =
\begin{bmatrix}
x_k \\ y_k \\ \theta_k
\end{bmatrix},
$$

kde $x_k$ a $y_k$ sú súradnice robota v globálnom rámci a $\theta_k$ je jeho
orientácia okolo osi $z$. Vstupný vektor diferenciálneho podvozka je

$$
\mathbf{u}_k =
\begin{bmatrix}
v_k \\ \omega_k
\end{bmatrix},
$$

kde $v_k$ je pozdĺžna rýchlosť a $\omega_k$ uhlová rýchlosť.

### 7.1 ICP registrácia skenov

Každý platný lúč LiDARu sa prevedie na bod v lokálnom rámci robota. Pre lúč
so vzdialenosťou $\rho_i$ a uhlom $\alpha_i$ platí

$$
\mathbf{p}_i =
\begin{bmatrix}
\rho_i\cos(\alpha_i) \\ \rho_i\sin(\alpha_i)
\end{bmatrix}.
$$

Nech $P_A=\{\mathbf{p}_{A,i}\}$ označuje aktuálny sken a
$P_B=\{\mathbf{p}_{B,i}\}$ referenčný sken alebo body známej mapy. Hľadaná
rovinná transformácia je

$$
\mathbf{p}_{B,i} \approx R(\theta)\mathbf{p}_{A,i}+\mathbf{t},
\qquad
\mathbf{t}=\begin{bmatrix}t_x\\t_y\end{bmatrix},
$$

pričom rotačná matica a jej derivácia sú

$$
R(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix},
\qquad
\frac{\partial R}{\partial\theta}=
\begin{bmatrix}
-\sin\theta & -\cos\theta \\
\cos\theta & -\sin\theta
\end{bmatrix}.
$$

Parametre transformácie sa určia minimalizáciou kvadratickej účelovej funkcie

$$
J(\theta,\mathbf{t})=
\frac{1}{2}\sum_{i=1}^{N}
\left\|R(\theta)\mathbf{p}_{A,i}+\mathbf{t}-\mathbf{p}_{B,i}\right\|^2.
$$

Pre aktuálny odhad $\hat{\theta}$ a $\hat{\mathbf{t}}$ je reziduum

$$
\mathbf{r}_i=
R(\hat{\theta})\mathbf{p}_{A,i}+
\hat{\mathbf{t}}-\mathbf{p}_{B,i}.
$$

Jacobián rezidua podľa parametrov
$\boldsymbol{\xi}=[\theta,t_x,t_y]^T$ je

$$
J_i=\frac{\partial\mathbf{r}_i}{\partial\boldsymbol{\xi}}=
\begin{bmatrix}
\frac{\partial R}{\partial\theta}\mathbf{p}_{A,i} & 1 & 0 \\
 & 0 & 1
\end{bmatrix}.
$$

Gaussov-Newtonov krok sa vypočíta zo sústavy

$$
H=\sum_i J_i^T J_i+\Lambda,\qquad
\mathbf{g}=\sum_i J_i^T\mathbf{r}_i,
$$

$$
\Delta\boldsymbol{\xi}=-H^{-1}\mathbf{g},
\qquad
\boldsymbol{\xi}_{j+1}=\boldsymbol{\xi}_j+\Delta\boldsymbol{\xi},
$$

kde $\Lambda$ je malé tlmiace číslo zlepšujúce numerickú stabilitu. V každej
ICP iterácii sa najprv transformujú body aktuálneho skenu, následne sa pre
každý bod vyhľadá najbližšia referencia a potom sa odhadne nový transformačný
parametr. Iterácie sa ukončia po dosiahnutí tolerancie, malej zmene parametrov
alebo po dosiahnutí maximálneho počtu iterácií. Korešpondencie s príliš veľkou
vzdialenosťou sa zamietnu.

V aktuálnej implementácii je referenciou predchádzajúci sken, takže ICP
poskytuje relatívny pohyb. Ak ICP nájde transformáciu aktuálneho skenu do
predchádzajúceho skenu $(R_{AB},\mathbf{t}_{AB})$, pohyb robota je daný
inverznou transformáciou

$$
R_{\mathrm{robot}}=R_{AB}^{T},\qquad
\mathbf{t}_{\mathrm{robot}}=-R_{AB}^{T}\mathbf{t}_{AB}.
$$

Pri použití známej okupačnej mapy je možné rovnaký postup interpretovať ako
priamy odhad globálnej transformácie skenu do mapového rámca.

### 7.2 Nelineárny model pohybu

Prechodová funkcia diferenciálneho podvozka je

$$
\mathbf{x}_{k+1}=f(\mathbf{x}_k,\mathbf{u}_k)=
\begin{bmatrix}
x_k+v_k\Delta t\cos\theta_k \\
y_k+v_k\Delta t\sin\theta_k \\
	heta_k+\omega_k\Delta t
\end{bmatrix}+\mathbf{w}_k,
$$

kde $\mathbf{w}_k$ je procesný šum s kovariančnou maticou $Q_k$. Jacobian
prechodovej funkcie podľa stavu je

$$
A_k=\frac{\partial f}{\partial\mathbf{x}}=
\begin{bmatrix}
1 & 0 & -v_k\Delta t\sin\theta_k \\
0 & 1 &  v_k\Delta t\cos\theta_k \\
0 & 0 & 1
\end{bmatrix}.
$$

### 7.3 Rozšírený Kalmanov filter

ICP poskytuje meranie polohy a orientácie v rovnakom stavovom priestore,
preto meracia funkcia a jej Jacobian majú tvar

$$
\mathbf{z}_k=h(\mathbf{x}_k)+\mathbf{v}_k=\mathbf{x}_k+\mathbf{v}_k,
\qquad
C_k=\frac{\partial h}{\partial\mathbf{x}}=I_3,
$$

kde $\mathbf{v}_k$ je merací šum s kovariančnou maticou $R_k$. EKF predikcia
z wheel odometrie je

$$
\hat{\mathbf{x}}_k^- = f(\hat{\mathbf{x}}_{k-1},\mathbf{u}_k),
\qquad
P_k^- = A_kP_{k-1}A_k^T+Q_k.
$$

Po prijatí ICP merania sa vypočíta inovačná kovariancia, Kalmanov zisk a nový
odhad:

$$
S_k=C_kP_k^-C_k^T+R_k,
\qquad
K_k=P_k^-C_k^TS_k^{-1},
$$

$$
\hat{\mathbf{x}}_k=\hat{\mathbf{x}}_k^-+
K_k\left(\mathbf{z}_k-h(\hat{\mathbf{x}}_k^-)\right),
$$

$$
P_k=(I-K_kC_k)P_k^-.
$$

Rozdiel uhlov sa pred aktualizáciou normalizuje do intervalu
$[-\pi,\pi]$, aby prechod cez hranicu $\pm\pi$ nevytvoril umelú veľkú chybu.
Kovariancie $Q_k$ a $R_k$ sú v implementácii nastaviteľné ROS parametre.

Node publikuje samostatne ICP odhad na
`/<robot_name>/icp/odometry` a filtrovaný odhad na
`/<robot_name>/ekf/odometry`. Počiatočný stav je $(0,0,0)$ v lokálnom rámci
`<robot_name>/odom`. Ground truth nie je vstupom filtra a slúži iba na spätné
vyhodnotenie presnosti.

## 8. Spustenie

Najprv sa načíta ROS 2 Jazzy a zostaví sa balík:

```bash
cd ~/Diploma/diploma_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select runner_sim
source install/setup.bash
```

Spustenie jedného robota:

```bash
ros2 launch runner_sim runner_sim.launch.py robot_count:=1
```

Predvolené spustenie používa skladové prostredie. Pre zobrazenie skenov v samostatnom RViz okne:

```bash
ros2 launch runner_sim runner_sim.launch.py robot_count:=1 rviz:=true
```

RViz zobrazuje predný pravý LiDAR oranžovou a zadný ľavý LiDAR modrou farbou. Dáta sú stále publikované na ROS témach a RViz ich iba vizualizuje; ground truth sa do lokalizačného vstupu nepridáva.

Pôvodné prázdne prostredie je možné spustiť explicitne:

```bash
ros2 launch runner_sim runner_sim.launch.py world:=runner_world.sdf
```

Spustenie šiestich robotov:

```bash
ros2 launch runner_sim runner_sim.launch.py robot_count:=6 spawn_spacing:=2.0
```

### 8.1 Režim publikovania senzorov

Predvolený režim je `localization_mode:=single`. V tomto režime sa pre
`runner_01` spustí bridge pre LiDAR, IMU, wheel odometriu a riadenie a zároveň
sa spustí ICP/EKF node. Pre ostatných robotov sa publikuje iba
`/<robot_name>/ground_truth/odometry`. Ich presná poloha tak môže predstavovať
znalosť polohy ostatných robotov bez zbytočného prenášania senzorových dát.

Pôvodné správanie, pri ktorom sa všetky vstupy a lokalizačná node spúšťajú pre
každého robota, sa aktivuje parametrom:

```bash
ros2 launch runner_sim runner_sim.launch.py \
  robot_count:=6 localization_mode:=all
```

Režim je dostupný aj v skripte `run_runner_sim.sh`, ktorý si ho vyžiada pri
spustení a použije `single`, ak sa ponechá predvolená hodnota.

Pri viacerých robotoch sa ich počiatočné polohy rozložia symetricky okolo stredu mapy. RViz konfigurácia zatiaľ zobrazuje LiDAR témy `/runner_01_lidar_front` a `/runner_01_lidar_rear`; pre porovnávanie ostatných robotov treba pridať ich témy alebo vytvoriť samostatné RViz konfigurácie.

Celý build a spustenie je možné vykonať pomocou skriptu z adresára `Diploma`:

```bash
cd ~/Diploma
./run_runner_sim.sh
```

Skript si vyžiada počet robotov a rozostup medzi ich počiatočnými pozíciami. Po spustení simulácie sa ukončí stlačením klávesu `q` v termináli, v ktorom bol skript spustený.

Po spustení treba overiť témy:

```bash
ros2 topic list | sort
ros2 topic echo /runner_01/sensors/imu/data
ros2 topic echo /runner_01_lidar_front
ros2 topic echo /runner_01/ground_truth/odometry
```

## 9. Dôležité otvorené úlohy

1. Zmerať skutočné rozmery tela, kolies, rozchod kolies a polohy ťažiska.
2. Zmerať presné polohy a orientácie oboch LiDARov voči základnému rámu robota.
3. Doplniť skutočný minimálny a maximálny dosah, uhlové pokrytie, rozlíšenie a frekvenciu LiDARov.
4. Získať šumové charakteristiky IMU a enkóderov z reálneho zariadenia.
5. Overiť hmotnosť, momenty zotrvačnosti, trenie a model preklzu.
6. Zaviesť načítanie parametrov z `runner_sim.yaml` priamo do Xacro alebo do simulačného launch súboru. Aktuálne je YAML dokumentačné centrálne miesto, ale nie všetky hodnoty sú z neho automaticky použité.
7. Overiť, že bridge pre ground truth používa iba hodnoty z Gazebo a nie je pripojený do lokalizačného uzla.
8. Implementovať maskovanie vlastného tela v LiDAR skenoch.
9. Otestovať vzájomné blokovanie LiDARov pri rôznych vzájomných polohách robotov v warehouse prostredí.
10. Pripraviť zaznamenávanie experimentov pomocou `ros2 bag` spolu s ground truth.
11. Definovať opakovateľné scenáre: jeden robot, viac robotov, prekážka, vzájomné blokovanie a výpadok senzora.
12. Pripraviť samostatný vyhodnocovací uzol alebo skript, ktorý porovnáva odhad s ground truth.

## 9. Témy vhodné do diplomovej práce

Do teoretickej a praktickej časti práce treba zahrnúť najmä:

- dôvod použitia simulácie pri vývoji lokalizačných algoritmov,
- model diferenciálneho pohonu,
- súradnicové rámy a transformácie v ROS 2,
- modelovanie LiDARu, IMU a enkóderov,
- model meracieho šumu a jeho vplyv na lokalizáciu,
- rozdiel medzi odometriou a ground truth,
- vzájomné tienenie a blokovanie LiDAR meraní robotmi,
- namespacing a správa viacerých identických robotov,
- návrh experimentálnych scenárov,
- metriky presnosti a spôsob vyhodnotenia,
- porovnanie simulovaných a reálnych meraní,
- obmedzenia simulácie a prenos výsledkov do reálneho robota.

## 10. Zásada pre ďalší vývoj

Každá zmena fyzických vlastností robota alebo senzorov má byť zapísaná do tejto dokumentácie a zároveň do konfiguračných súborov. Pri každom experimente treba zaznamenať verziu modelu, počet robotov, počiatočné polohy, použité šumové parametre a to, ktoré témy boli dostupné lokalizačnému algoritmu.

Ground truth sa musí uchovávať oddelene od meraní počas celého experimentu. Tým sa zabezpečí, že výsledná chyba lokalizácie bude reprodukovateľná a že algoritmus nebude neúmyselne využívať skutočnú polohu robota.

## 11. Kalman odhadu z IMU -- important aj do diplomovky
pri navrhu kalmana je nune vytvorit statisticky optimalny odhad premennej kalmana K(k)(y(k)-Cx_hat(k|k-1)).
Cely kalmanov filter pre IMU budem vkladat z Dodekovho premetu z cvika v stredu

## 12. lokalizacia amcl ros2 package

## 13. mergovanie scanu
mozno da, je to iba jedna transformacia

## 14. misia do kruhu 
zmenit .sh skript aby subscriboval iba konkretny runner, mat moznost lidarovy

## co najskor na github