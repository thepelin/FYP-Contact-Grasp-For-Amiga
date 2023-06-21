import numpy as np
import pandas as pd
import statsmodels.api as sm
from statsmodels.formula.api import ols
import matplotlib.pyplot as plt
from scipy.stats import pearsonr


success_list = []
time_list = []
total_poses_list = []
executed_pose_list = []

data = """
0
Success: True
Time: 186.4004464149475
Total # of poses: 11
Executed pose #: 4
1
Success: True
Time: 271.248735666275
Total # of poses: 10
Executed pose #: 6
2
Success: True
Time: 141.06569528579712
Total # of poses: 15
Executed pose #: 5
3
Success: True
Time: 33.827253341674805
Total # of poses: 17
Executed pose #: 1
4
Success: True
Time: 87.27654075622559
Total # of poses: 14
Executed pose #: 2
5
Success: True
Time: 172.60557103157043
Total # of poses: 12
Executed pose #: 5
6
Success: True
Time: 192.76703143119812
Total # of poses: 11
Executed pose #: 1
7
Success: True
Time: 55.95048975944519
Total # of poses: 11
Executed pose #: 1
8
Success: True
Time: 550.4938282966614
Total # of poses: 11
Executed pose #: 1
9
Success: True
Time: 135.15701270103455
Total # of poses: 11
Executed pose #: 3
10
Success: True
Time: 136.45752811431885
Total # of poses: 12
Executed pose #: 3
11
Success: True
Time: 367.8685975074768
Total # of poses: 10
Executed pose #: 9
12
Success: True
Time: 105.60829639434814
Total # of poses: 11
Executed pose #: 4
13
Success: True
Time: 247.5098717212677
Total # of poses: 12
Executed pose #: 7
14
Success: True
Time: 131.0657238960266
Total # of poses: 11
Executed pose #: 3
15
Success: True
Time: 195.1705162525177
Total # of poses: 18
Executed pose #: 6
16
Success: True
Time: 67.86184978485107
Total # of poses: 18
Executed pose #: 1
17
Success: False
Time: 537.6697299480438
Total # of poses: 11
Executed pose #: 11
18
Success: True
Time: 278.05137157440186
Total # of poses: 17
Executed pose #: 5
19
Success: True
Time: 102.72050404548645
Total # of poses: 12
Executed pose #: 2
20
Success: True
Time: 130.74414896965027
Total # of poses: 11
Executed pose #: 2
21
Success: True
Time: 375.70818734169006
Total # of poses: 19
Executed pose #: 2
22
Success: True
Time: 48.0110547542572
Total # of poses: 15
Executed pose #: 1
23
Success: False
Time: 929.9042918682098
Total # of poses: 15
Executed pose #: 15
24
Success: True
Time: 175.54218339920044
Total # of poses: 23
Executed pose #: 1
25
Success: True
Time: 162.65010905265808
Total # of poses: 21
Executed pose #: 3
26
Success: True
Time: 79.75545191764832
Total # of poses: 19
Executed pose #: 1
27
Success: True
Time: 121.63886880874634
Total # of poses: 17
Executed pose #: 2
28
Success: True
Time: 382.2815511226654
Total # of poses: 14
Executed pose #: 12
29
Success: True
Time: 75.15852379798889
Total # of poses: 16
Executed pose #: 2

Success: True
Time: 97.81913208961487
Total # of poses: 70
Executed pose #: 2
1
Success: True
Time: 91.56160736083984
Total # of poses: 70
Executed pose #: 3
0
Success: True
Time: 1982.5355515480042
Total # of poses: 58
Executed pose #: 17
0
Success: True
Time: 195.920081615448
Total # of poses: 75
Executed pose #: 5
1
Success: True
Time: 36.61380481719971
Total # of poses: 66
Executed pose #: 1
2
Success: True
Time: 34.016475200653076
Total # of poses: 73
Executed pose #: 1
3
Success: True
Time: 33.687216997146606
Total # of poses: 63
Executed pose #: 1
4
Success: True
Time: 67.01757001876831
Total # of poses: 75
Executed pose #: 2
5
Success: False
Time: 691.1653337478638
Total # of poses: 67
Executed pose #: 21
6
Success: True
Time: 101.69354629516602
Total # of poses: 83
Executed pose #: 2
7
Success: True
Time: 235.01318192481995
Total # of poses: 80
Executed pose #: 7
8
Success: False
Time: 756.8859069347382
Total # of poses: 83
Executed pose #: 21
9
Success: True
Time: 1245.3294639587402
Total # of poses: 83
Executed pose #: 1
0
Success: True
Time: 192.46600365638733
Total # of poses: 62
Executed pose #: 6
1
Success: True
Time: 28.390685558319092
Total # of poses: 58
Executed pose #: 1
0
Success: True
Time: 226.57614636421204
Total # of poses: 64
Executed pose #: 12
0
Success: True
Time: 31.318745136260986
Total # of poses: 74
Executed pose #: 1
1
Success: True
Time: 64.44146275520325
Total # of poses: 62
Executed pose #: 2
2
Success: True
Time: 66.90201306343079
Total # of poses: 73
Executed pose #: 2
3
Success: True
Time: 35.733327865600586
Total # of poses: 76
Executed pose #: 1
4
Success: True
Time: 30.99247431755066
Total # of poses: 74
Executed pose #: 1
5
Success: True
Time: 32.80080842971802
Total # of poses: 61
Executed pose #: 1
6
Success: True
Time: 33.25328350067139
Total # of poses: 69
Executed pose #: 1
7
Success: True
Time: 357.0908329486847
Total # of poses: 63
Executed pose #: 10
8
Success: True
Time: 98.46224212646484
Total # of poses: 72
Executed pose #: 3
9
Success: True
Time: 39.1990602016449
Total # of poses: 62
Executed pose #: 1
10
Success: True
Time: 32.53449892997742
Total # of poses: 65
Executed pose #: 1
11
Success: True
Time: 84.13474678993225
Total # of poses: 63
Executed pose #: 2
12
Success: True
Time: 43.752832651138306
Total # of poses: 65
Executed pose #: 1
13
Success: True
Time: 74.3759434223175
Total # of poses: 73
Executed pose #: 2
0
Success: True
Time: 106.24817276000977
Total # of poses: 15
Executed pose #: 2
1
Success: True
Time: 105.9386830329895
Total # of poses: 14
Executed pose #: 3
2
Success: True
Time: 142.98798537254333
Total # of poses: 11
Executed pose #: 3
3
Success: True
Time: 138.99315905570984
Total # of poses: 10
Executed pose #: 3
4
Success: False
Time: 645.6632328033447
Total # of poses: 17
Executed pose #: 17
5
Success: True
Time: 268.06438851356506
Total # of poses: 15
Executed pose #: 2
6
Success: True
Time: 46.944751024246216
Total # of poses: 10
Executed pose #: 1
7
Success: True
Time: 210.75483870506287
Total # of poses: 13
Executed pose #: 4
8
Success: False
Time: 505.3317139148712
Total # of poses: 12
Executed pose #: 12
9
Success: True
Time: 268.6984040737152
Total # of poses: 10
Executed pose #: 1
10
Success: True
Time: 37.02958345413208
Total # of poses: 16
Executed pose #: 1
11
Success: True
Time: 96.67299556732178
Total # of poses: 12
Executed pose #: 4
12
Success: True
Time: 95.3369574546814
Total # of poses: 10
Executed pose #: 2
13
Success: False
Time: 416.9169924259186
Total # of poses: 11
Executed pose #: 11
14
Success: False
Time: 439.58691811561584
Total # of poses: 10
Executed pose #: 10
15
Success: True
Time: 210.20408034324646
Total # of poses: 14
Executed pose #: 4
16
Success: True
Time: 516.5273659229279
Total # of poses: 23
Executed pose #: 15
17
Success: True
Time: 167.47303080558777
Total # of poses: 24
Executed pose #: 4
18
Success: True
Time: 96.73103928565979
Total # of poses: 19
Executed pose #: 3
19
Success: True
Time: 110.93515229225159
Total # of poses: 11
Executed pose #: 2
20
Success: True
Time: 99.58203172683716
Total # of poses: 11
Executed pose #: 2
21
Success: False
Time: 609.8817622661591
Total # of poses: 14
Executed pose #: 14
22
Success: True
Time: 41.91594457626343
Total # of poses: 29
Executed pose #: 1
23
Success: True
Time: 41.19393515586853
Total # of poses: 27
Executed pose #: 1
24
Success: False
Time: 520.367288351059
Total # of poses: 14
Executed pose #: 14
25
Success: False
Time: 446.88510370254517
Total # of poses: 10
Executed pose #: 10
26
Success: True
Time: 357.8759706020355
Total # of poses: 11
Executed pose #: 7
27
Success: True
Time: 43.92263078689575
Total # of poses: 15
Executed pose #: 1
28
Success: False
Time: 381.8769302368164
Total # of poses: 10
Executed pose #: 10
29
Success: True
Time: 352.72734355926514
Total # of poses: 13
Executed pose #: 2
0
Success: False
Time: 1737.2051978111267
Total # of poses: 74
Executed pose #: 21
1
Success: True
Time: 103.60104632377625
Total # of poses: 63
Executed pose #: 5
2
Success: True
Time: 18.347463607788086
Total # of poses: 69
Executed pose #: 1
3
Success: True
Time: 141.95666122436523
Total # of poses: 67
Executed pose #: 5
4
Success: True
Time: 25.526113986968994
Total # of poses: 58
Executed pose #: 3
5
Success: True
Time: 19.323875188827515
Total # of poses: 69
Executed pose #: 1
6
Success: True
Time: 67.51554012298584
Total # of poses: 57
Executed pose #: 2
7
Success: True
Time: 150.43406438827515
Total # of poses: 64
Executed pose #: 7
8
Success: True
Time: 19.80138111114502
Total # of poses: 64
Executed pose #: 1
9
Success: True
Time: 56.3384370803833
Total # of poses: 62
Executed pose #: 3
10
Success: True
Time: 203.94647789001465
Total # of poses: 75
Executed pose #: 7
11
Success: True
Time: 77.52479076385498
Total # of poses: 48
Executed pose #: 5
12
Success: True
Time: 20.06740927696228
Total # of poses: 62
Executed pose #: 2
13
Success: True
Time: 20.160564661026
Total # of poses: 66
Executed pose #: 1
14
Success: True
Time: 23.393652200698853
Total # of poses: 65
Executed pose #: 1
15
Success: True
Time: 28.333868741989136
Total # of poses: 50
Executed pose #: 2
16
Success: True
Time: 19.92417597770691
Total # of poses: 48
Executed pose #: 1
17
Success: True
Time: 27.106874465942383
Total # of poses: 64
Executed pose #: 4
18
Success: True
Time: 19.93815851211548
Total # of poses: 57
Executed pose #: 3
19
Success: True
Time: 18.956003665924072
Total # of poses: 53
Executed pose #: 1
20
Success: True
Time: 53.818294286727905
Total # of poses: 66
Executed pose #: 2
21
Success: True
Time: 18.9593403339386
Total # of poses: 63
Executed pose #: 1
22
Success: True
Time: 26.19149923324585
Total # of poses: 66
Executed pose #: 3
23
Success: True
Time: 20.381104230880737
Total # of poses: 65
Executed pose #: 1
24
Success: True
Time: 19.6253445148468
Total # of poses: 57
Executed pose #: 1
25
Success: True
Time: 22.376578330993652
Total # of poses: 65
Executed pose #: 1
26
Success: True
Time: 171.56936168670654
Total # of poses: 63
Executed pose #: 5
27
Success: True
Time: 20.1136155128479
Total # of poses: 61
Executed pose #: 2
28
Success: True
Time: 61.084603786468506
Total # of poses: 62
Executed pose #: 2
29
Success: True
Time: 115.68226599693298
Total # of poses: 68
Executed pose #: 6
"""

lines = data.strip().split('\n')

for i in range(0, len(lines), 5):
    value = lines[i+1].split(': ')[1]
    success = 1 if value == "True" else 0
    time = float(lines[i+2].split(': ')[1])
    total_poses = int(lines[i+3].split(': ')[1])
    executed_pose = int(lines[i+4].split(': ')[1])
    
    success_list.append(success)
    time_list.append(time)
    total_poses_list.append(total_poses)
    executed_pose_list.append(executed_pose)

print("Success:", success_list)
print("Time:", time_list)
print("Total # of poses:", total_poses_list)
print("Executed pose #:", executed_pose_list)

df = pd.DataFrame({'X': np.repeat(['C', 'U'], 60),
                   'Y': np.tile(np.repeat(['B', 'K'], 30), 2),
                   'success_list': success_list})

#perform three-way ANOVA
model = ols("""success_list  ~ C(X) + C(Y) +
               C(X):C(Y)""", data=df).fit()

print("Success:")
print(sm.stats.anova_lm(model, typ=2))


df = pd.DataFrame({'X': np.repeat(['C', 'U'], 60),
                   'Y': np.tile(np.repeat(['B', 'K'], 30), 2),
                   'time_list': time_list})

#perform three-way ANOVA
model = ols("""time_list  ~ C(X) + C(Y) +
               C(X):C(Y)""", data=df).fit()
print("Time:")
print(sm.stats.anova_lm(model, typ=2))


df = pd.DataFrame({'X': np.repeat(['C', 'U'], 60),
                   'Y': np.tile(np.repeat(['B', 'K'], 30), 2),
                   'total_poses_list': total_poses_list})

#perform three-way ANOVA
model = ols("""total_poses_list ~ C(X) + C(Y) +
               C(X):C(Y)""", data=df).fit()

print("Total # of poses:")
print(sm.stats.anova_lm(model, typ=2))


df = pd.DataFrame({'X': np.repeat(['C', 'U'], 60),
                   'Y': np.tile(np.repeat(['B', 'K'], 30), 2),
                   'executed_pose_list': executed_pose_list})

#perform three-way ANOVA
model = ols("""executed_pose_list ~ C(X) + C(Y) +
               C(X):C(Y)""", data=df).fit()

print("Executed pose #:")
print(sm.stats.anova_lm(model, typ=2))

success_sublists = [success_list[i:i+30] for i in range(0, len(success_list), 30)]
print("ACCURACY RATE:", [np.mean(d) for d in success_sublists])

time_sublists = [time_list[i:i+30] for i in range(0, len(time_list), 30)]

# Combine the data
data = time_sublists

# Create the plot
fig, ax = plt.subplots()

# Create box plots
box_plot = ax.boxplot(data, vert=False)

# Add labels and title
ax.set_yticklabels(['Bottle, Changed', 'Bowl, Changed', 'Bottle, Unchanged','Bowl, Unchanged'])
ax.set_ylabel('Dataset')
ax.set_xlabel('Value')
ax.set_title('Distribution of Time Data')

# Add mean indicators
mean_values = [np.mean(d) for d in data]
positions = range(1, len(data) + 1)
ax.plot(mean_values, positions, marker='o', linestyle='', color='red', label='Mean')

# Add median indicators
median_values = [np.median(d) for d in data]
ax.plot(median_values, positions, marker='s', linestyle='', color='green', label='Median')

# Add legend
ax.legend()

# Display the plot
plt.savefig('time_dis_turn.png', dpi=300, bbox_inches='tight')


pose_sublists = [total_poses_list[i:i+30] for i in range(0, len(total_poses_list), 30)]

# Combine the data
data = pose_sublists

# Create the plot
fig, ax = plt.subplots()

# Create box plots
box_plot = ax.boxplot(data, vert=False)

# Add labels and title
ax.set_yticklabels(['Bottle, Changed', 'Bowl, Changed', 'Bottle, Unchanged','Bowl, Unchanged'])
ax.set_ylabel('Dataset')
ax.set_xlabel('Value')
ax.set_title('Distribution of Total Pose Data')

# Add mean indicators
mean_values = [np.mean(d) for d in data]
positions = range(1, len(data) + 1)
ax.plot(mean_values, positions, marker='o', linestyle='', color='red', label='Mean')

# Add median indicators
median_values = [np.median(d) for d in data]
ax.plot(median_values, positions, marker='s', linestyle='', color='green', label='Median')

# Add legend
ax.legend()

# Display the plot
plt.savefig('total_pose_dis_turn.png', dpi=300, bbox_inches='tight')


ex_sublists = [executed_pose_list[i:i+30] for i in range(0, len(executed_pose_list), 30)]

# Combine the data
data = ex_sublists

# Create the plot
fig, ax = plt.subplots()

# Create box plots
box_plot = ax.boxplot(data, vert=False)

# Add labels and title
ax.set_yticklabels(['Bottle, Changed', 'Bowl, Changed', 'Bottle, Unchanged','Bowl, Unchanged'])
ax.set_ylabel('Dataset')
ax.set_xlabel('Value')
ax.set_title('Distribution of Executed Pose Data')

# Add mean indicators
mean_values = [np.mean(d) for d in data]
positions = range(1, len(data) + 1)
ax.plot(mean_values, positions, marker='o', linestyle='', color='red', label='Mean')

# Add median indicators
median_values = [np.median(d) for d in data]
ax.plot(median_values, positions, marker='s', linestyle='', color='green', label='Median')

# Add legend
ax.legend()

# Display the plot
plt.savefig('ex_pose_dis_turn.png', dpi=300, bbox_inches='tight')


# print("Success:", success_list)
# print("Time:", time_list)
# print("Total # of poses:", total_poses_list)
# print("Executed pose #:", executed_pose_list)

results = pd.DataFrame({"Accuracy": success_list,
                        "Time": time_list,
                        "Total_Poses": total_poses_list,
                        "Executed_Pose": executed_pose_list})

correlation_matrix = results.corr()
print(correlation_matrix)

data = np.array([success_list, time_list, total_poses_list, executed_pose_list])


correlation_matrix = np.corrcoef(data)
p_values = np.zeros_like(correlation_matrix)
corr_values = np.zeros_like(correlation_matrix)

# Calculate p-values
num_vars = data.shape[0]
for i in range(num_vars):
    for j in range(num_vars):
        corr, p_val = pearsonr(data[i,:], data[j,:])
        corr_values[i, j] = corr
        corr_values[j, i] = corr

        p_values[i, j] = p_val
        p_values[j, i] = p_val

print("Correlation matrix:")
print(corr_values)

print("P-values:")
print(p_values)