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

Oba LiDARy sú umiestnené vo výške 0,85 m nad zemou (v modeli
`lidar_height - body_z` voči `base_link`, ktorý leží na úrovni podlahy), teda
pod hornou plochou robotického kvádra (1,5 m). Vďaka tomu LiDAR vidí aj
ostatných runnerov. Ich pôdorysné súradnice sú mimo rozmerov kvádra, aby
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

Plugin diferenciálneho pohonu publikuje odometriu kolies. Táto odometria nie je ground truth; v aktuálnej verzii sa nepoužíva ani na riadenie, ani v EKF (pozri kapitolu 7), zostáva dostupná na porovnanie. V ďalšej fáze sa musí overiť, či simulované chyby odometrie dostatočne reprezentujú chyby reálneho robota, najmä:

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
| `/runner_01/icp/odometry` | `nav_msgs/msg/Odometry` | LiDAR odometria (integrované ICP) |
| `/runner_01/ekf/odometry` | `nav_msgs/msg/Odometry` | odhad EKF |
| `/visualization_marker_array` | `visualization_msgs/msg/MarkerArray` | obrysy robotov a elipsa neistoty `runner_01` |
| `/clock` | `rosgraph_msgs/msg/Clock` | simulačný čas pre všetky node s `use_sim_time` |

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
`lidar_localization_node`. Robot sa riadi podľa ground truth (pozri kapitolu 9),
lokalizácia preto beží nezávisle a jej odhad sa iba porovnáva so skutočnou
polohou. EKF odhaduje stavový vektor

$$
\mathbf{x}_k =
\begin{bmatrix}
x_k & y_k & \theta_k & v_{x,k} & v_{y,k} & \omega_k
\end{bmatrix}^T,
$$

kde $x_k$, $y_k$ a $\theta_k$ sú poloha a orientácia robota v lokálnom rámci
`<robot_name>/odom` a $v_x$, $v_y$, $\omega$ sú rýchlosti v rámci robota
(`base_link`). Wheel odometria nie je vstupom filtra, pretože v Gazebo je
takmer bezchybná a zakryla by chyby LiDAR odometrie, ktoré sú predmetom práce.

Filter má dva režimy, ktoré sa volia pri spustení parametrom `ekf_mode`:

| Režim | Vstupy EKF |
|---|---|
| `ICP` | iba LiDAR odometria (ICP) |
| `IMU_ICP` | LiDAR odometria (ICP) + uhlová rýchlosť z gyroskopu IMU |

### 7.1 Predspracovanie skenu

Oba LiDARy sú umiestnené mimo rohov kvádra robota, preto časť lúčov dopadá na
vlastné telo robota. Tieto body sa pohybujú spolu s robotom a ťahali by
odhad ICP k nulovému pohybu. Pri zlučovaní skenov sa preto odstránia všetky
body vnútri pôdorysu robota zväčšeného o rezervu `self_filter_margin`
(predvolene 0,1 m). Body ostatných robotov sa zámerne **neodstraňujú**,
pretože ich vplyv na lokalizáciu je predmetom skúmania.

### 7.2 ICP registrácia skenov (point-to-line)

Každý platný lúč zlúčeného skenu sa prevedie na bod v rámci robota

$$
\mathbf{p}_i =
\begin{bmatrix}
\rho_i\cos(\alpha_i) \\ \rho_i\sin(\alpha_i)
\end{bmatrix}.
$$

Nech $P_A=\{\mathbf{p}_{A,i}\}$ je aktuálny sken a $P_B=\{\mathbf{p}_{B,j}\}$
predchádzajúci (referenčný) sken. Pre každý referenčný bod sa z jeho susedov
(metóda hlavných komponentov v okne $\pm 2$ lúčov) určí normála povrchu
$\mathbf{n}_j$. Hľadá sa transformácia

$$
\mathbf{p}_{B,j} \approx R(\theta)\mathbf{p}_{A,i}+\mathbf{t},
$$

ktorá minimalizuje vzdialenosti bodov od referenčných priamok (point-to-line):

$$
r_i=\mathbf{n}_j^T\left(R(\theta)\mathbf{p}_{A,i}+\mathbf{t}-\mathbf{p}_{B,j}\right),
\qquad
J(\theta,\mathbf{t})=\frac{1}{2}\sum_i w_i r_i^2 ,
$$

kde $j$ je najbližší referenčný bod (vyhľadaný v pravidelnej mriežke s
maximálnou vzdialenosťou korešpondencie) a $w_i$ je Huberova váha
$w_i=\min(1, k/|r_i|)$. Jacobián rezidua podľa
$\boldsymbol{\xi}=[\theta,t_x,t_y]^T$ je

$$
J_i=
\begin{bmatrix}
\mathbf{n}_j^T\dfrac{\partial R}{\partial\theta}\mathbf{p}_{A,i} & n_{j,x} & n_{j,y}
\end{bmatrix},
$$

a Gaussov-Newtonov krok

$$
H=\sum_i w_iJ_i^TJ_i,\qquad
\mathbf{g}=\sum_i w_iJ_i^Tr_i,\qquad
\Delta\boldsymbol{\xi}=-H^{-1}\mathbf{g}.
$$

Počiatočný odhad ICP je pohyb predikovaný z rýchlostí EKF
($[v_x\Delta t,\ v_y\Delta t,\ \omega\Delta t]$); v režime `IMU_ICP` je tak
otočenie podporené gyroskopom.

Transformácia nájdená ICP mapuje body aktuálneho skenu do rámca
predchádzajúceho skenu. Ak $T_k$ je poloha robota v čase $k$, platí
$\mathbf{p}_{B}=T_{k-1}^{-1}T_k\,\mathbf{p}_{A}$, takže výsledok
$(R(\theta),\mathbf{t})$ je **priamo** relatívny pohyb robota medzi skenmi
(nie jeho inverzia):

$$
\Delta\mathbf{x}_k=\begin{bmatrix}t_x & t_y & \theta\end{bmatrix}^T .
$$

Kovariancia odhadu sa určí z Gaussovho-Newtonovho Hessiánu:

$$
\Sigma_{\mathrm{ICP}} = s\,\hat\sigma^2 H^{-1} + \Sigma_{\min},
\qquad
\hat\sigma^2=\frac{1}{N}\sum_i r_i^2 ,
$$

kde $s$ (`icp_covariance_scale`) a $\Sigma_{\min}$ kompenzujú známu
prílišnú sebadôveru tohto odhadu. Pri degenerovanej geometrii (napr. dlhá
chodba) má $H$ malé vlastné číslo a kovariancia v neurčenom smere rastie.
Výsledok sa zamietne, ak je podiel korešpondencií menší ako
`icp_min_inlier_ratio` alebo je krok nereálne veľký.

### 7.3 Model pohybu (konštantná rýchlosť)

$$
f(\mathbf{x},\Delta t)=
\begin{bmatrix}
x+(v_x\cos\theta-v_y\sin\theta)\Delta t \\
y+(v_x\sin\theta+v_y\cos\theta)\Delta t \\
\theta+\omega\Delta t \\
v_x \\ v_y \\ \omega
\end{bmatrix},
\qquad
P^-=A P A^T + Q,
$$

kde $A=\partial f/\partial\mathbf{x}$ a procesný šum je biely šum zrýchlení
$Q=\mathrm{diag}(0,0,0,\sigma_{a_x}^2,\sigma_{a_y}^2,\sigma_{\alpha}^2)\Delta t$.

### 7.4 Merania EKF

**LiDAR odometria.** Relatívny pohyb z ICP za interval $\Delta t$ medzi skenmi
sa vkladá ako priemerná rýchlosť v rámci robota:

$$
\mathbf{z}_{\mathrm{ICP}}=\frac{\Delta\mathbf{x}_k}{\Delta t},
\qquad
C_{\mathrm{ICP}}=
\begin{bmatrix}
0_{3\times3} & I_3
\end{bmatrix},
\qquad
R_{\mathrm{ICP}}=\frac{\Sigma_{\mathrm{ICP}}}{\Delta t^2}.
$$

**Gyroskop (iba `IMU_ICP`).** $z_{\mathrm{IMU}}=\omega_z$,
$C_{\mathrm{IMU}}=[0\ 0\ 0\ 0\ 0\ 1]$,
$R_{\mathrm{IMU}}=\sigma_g^2$.

Aktualizácia je štandardná, kovariancia sa počíta v Josephovom tvare:

$$
S=CP^-C^T+R,\qquad K=P^-C^TS^{-1},\qquad
\hat{\mathbf{x}}=\hat{\mathbf{x}}^-+K(\mathbf{z}-C\hat{\mathbf{x}}^-),
$$

$$
P=(I-KC)P^-(I-KC)^T+KRK^T .
$$

Voliteľne je možné zapnúť Mahalanobisovo hradlo
$(\mathbf{z}-C\hat{\mathbf{x}}^-)^TS^{-1}(\mathbf{z}-C\hat{\mathbf{x}}^-)
<\gamma$ parametrom `icp_mahalanobis_gate` (predvolene vypnuté, aby sa chybné
ICP merania prejavili vo filtri).

Keďže filter nemá absolútne merania polohy, kovariancia polohy počas jazdy
rastie. Ak ICP ovplyvnia pohybujúce sa objekty (iní runneri), odhad sa
systematicky odchýli, no kovariancia to nemusí zachytiť – filter sa stane
prehnane sebaistým. Tento jav je viditeľný v RViz: skutočná poloha (zelená)
leží mimo elipsy neistoty.

### 7.5 Výstupy

| Téma | Obsah |
|---|---|
| `/<robot_name>/icp/odometry` | iba integrovaná ICP odometria (poloha + kovariancia) |
| `/<robot_name>/ekf/odometry` | odhad EKF: poloha, rýchlosti a ich kovariancie |
| `/<robot_name>/lidar_fused` | zlúčený 360° sken bez bodov vlastného robota |

Počiatočný stav je $(0,0,0)$ v rámci `<robot_name>/odom`; vizualizačná node
ho posunie o počiatočnú polohu robota do rámca `world`. Ground truth nie je
vstupom filtra.

### 7.6 Elipsa neistoty v RViz

Pre `runner_01` sa publikuje marker `runner_01/covariance_ellipse` na téme
`/visualization_marker_array`. Z kovariancie polohy $P_{xy}$ (otočenej do
rámca `world`) sa určia vlastné čísla $\lambda_{1,2}$ a uhol hlavnej osi
$\varphi=\tfrac12\operatorname{atan2}(2P_{xy},P_{xx}-P_{yy})$. Pre
pravdepodobnosť $p$ (`covariance_ellipse_confidence`, predvolene 0,95) majú
poloosi dĺžku

$$
a_{1,2}=\sqrt{-2\ln(1-p)}\,\sqrt{\lambda_{1,2}} .
$$

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
ros2 launch runner_sim runner_sim.launch.py robot_count:=6
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

## 9. Misie, riadenie a experiment s rušením

Runnery sa riadia podľa **ground truth** (`/<robot_name>/ground_truth/odometry`),
nie podľa odhadu EKF. Lokalizácia tak neovplyvňuje trajektóriu a rôzne režimy
EKF je možné porovnávať na rovnakom pohybe. Súradnice v CSV misiách sú v
rámci `world` (spoločný pre všetkých runnerov). Runnery sa spawnujú vo formácii
v radoch po troch, natočené v smere $+x$ (k severnej stene na $x=11$),
s rozostupom stĺpcov 1,2 m v $y$ (susedné runnery sa môžu súčasne otočiť na mieste,
polomer otáčania je približne 0,58 m) a radov 1,3 m v $x$ (medzera 0,3 m).
Stĺpce ležia vo voľnom koridore medzi regálmi `regal4`/`regal5` ($y \le -0{,}95$)
a regálmi na $y \ge 2{,}95$, takže runnery môžu ísť rovno vpred až k stene bez kolízie.
Pohľad zozadu (v smere $+x$):

```text
runner_06 (-0.7, 2.2)    runner_04 (-0.7, 1.0)    runner_05 (-0.7,-0.2)
runner_03 (-2.0, 2.2)    runner_01 (-2.0, 1.0)    runner_02 (-2.0,-0.2)
```

Gazebo server sa spúšťa samostatne (`-s`) a GUI (`gz sim -g`) sa otvorí až po
úspešnom spawne všetkých runnerov. GUI spustené počas spawnovania niektoré runnery
nezobrazilo, hoci v simulácii existovali; GUI spustené neskôr načíta celú scénu.

Formát riadku misie:

```text
runner_number,x_m,y_m,circle_radius_m,speed_percent,hard_point[,start_delay_s]
```

Voliteľný 7. stĺpec `start_delay_s` oneskorí odoslanie príkazu (v simulačnom
čase) od chvíle, keď sa riadok stane aktuálnym. Riadky začínajúce `#` sú
komentáre.

Pre porovnanie vplyvu ostatných runnerov sú pripravené dve misie pre 3 runnerov
s rovnakou trajektóriou `runner_01`:

- `mission_03_baseline.csv` – `runner_02` a `runner_03` zaparkujú mimo uličky a
  stoja (iba statické prekážky),
- `mission_03_interference.csv` – `runner_02` ide pred `runner_01` rovnakým
  smerom a `runner_03` ho míňa v protismere v susednom pruhu.

Orientačné výsledky (maximálna chyba polohy `runner_01` voči ground truth):

| Misia | `ICP` | `IMU_ICP` |
|---|---|---|
| baseline | 0,19 m | 0,20 m |
| interference | 1,77 m | 1,73 m |

Runner idúci vpredu spôsobí, že ICP podhodnocuje dopredný pohyb. IMU zlepší
orientáciu a bočný drift, ale pozdĺžnu chybu neodstráni, pretože gyroskop
neobsahuje informáciu o translácii.

Spustenie bez GUI (napr. pre dávkové experimenty):

```bash
ros2 launch runner_sim runner_sim.launch.py robot_count:=3 run_mission:=true \
  mission_file:=$PWD/install/runner_sim/share/runner_sim/missions/mission_03_interference.csv \
  ekf_mode:=IMU_ICP gz_extra_args:="-s --headless-rendering"
```
