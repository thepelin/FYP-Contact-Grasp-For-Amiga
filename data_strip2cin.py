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
0
Success: True
Time: 20.70390486717224
Total # of poses: 20
Executed pose #: 1
1
Success: True
Time: 182.4950590133667
Total # of poses: 9
Executed pose #: 5
2
Success: False
Time: 254.11219549179077
Total # of poses: 6
Executed pose #: 6
3
Success: True
Time: 338.5837256908417
Total # of poses: 6
Executed pose #: 6
4
Success: True
Time: 33.349796772003174
Total # of poses: 7
Executed pose #: 1
5
Success: True
Time: 34.55036473274231
Total # of poses: 6
Executed pose #: 1
6
Success: False
Time: 242.57400798797607
Total # of poses: 6
Executed pose #: 6
7
Success: True
Time: 37.852264642715454
Total # of poses: 7
Executed pose #: 1
8
Success: True
Time: 37.36732053756714
Total # of poses: 8
Executed pose #: 1
9
Success: True
Time: 35.325592279434204
Total # of poses: 6
Executed pose #: 1
10
Success: True
Time: 96.55923199653625
Total # of poses: 7
Executed pose #: 2
11
Success: False
Time: 159.21507740020752
Total # of poses: 3
Executed pose #: 3
12
Success: True
Time: 23.463181734085083
Total # of poses: 8
Executed pose #: 1
13
Success: False
Time: 252.68540167808533
Total # of poses: 5
Executed pose #: 5
14
Success: True
Time: 206.34030866622925
Total # of poses: 13
Executed pose #: 6
15
Success: True
Time: 105.48940062522888
Total # of poses: 16
Executed pose #: 3
16
Success: True
Time: 92.84379839897156
Total # of poses: 14
Executed pose #: 2
17
Success: True
Time: 165.30059504508972
Total # of poses: 13
Executed pose #: 5
18
Success: False
Time: 97.74408292770386
Total # of poses: 3
Executed pose #: 3
19
Success: True
Time: 186.43730235099792
Total # of poses: 8
Executed pose #: 5
20
Success: True
Time: 60.03346538543701
Total # of poses: 8
Executed pose #: 1
21
Success: True
Time: 25.636994123458862
Total # of poses: 8
Executed pose #: 1
22
Success: True
Time: 25.871328115463257
Total # of poses: 8
Executed pose #: 1
23
Success: True
Time: 49.79250764846802
Total # of poses: 14
Executed pose #: 5
24
Success: True
Time: 188.33024263381958
Total # of poses: 8
Executed pose #: 5
25
Success: True
Time: 36.73431205749512
Total # of poses: 7
Executed pose #: 1
26
Success: True
Time: 23.9711012840271
Total # of poses: 9
Executed pose #: 1
27
Success: True
Time: 40.97286915779114
Total # of poses: 8
Executed pose #: 1
28
Success: False
Time: 273.380975484848
Total # of poses: 6
Executed pose #: 6
29
Success: True
Time: 33.2081778049469
Total # of poses: 11
Executed pose #: 2
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

from scipy.stats import f_oneway

# Define your data
independent_variable = np.array([0, 0, 1, 1])  # Independent variable
dependent_variable1 = np.array([10, 12, 15, 8])  # First dependent variable
dependent_variable2 = np.array([5, 7, 6, 9])  # Second dependent variable
dependent_variable3 = np.array([3, 4, 5, 2])  # Third dependent variable

# Perform one-way ANOVA
f_value, p_value = f_oneway(dependent_variable1, dependent_variable2, dependent_variable3)

# Print the results
print("One-Way ANOVA")
print("F-value:", f_value)
print("p-value:", p_value)


# data = pd.DataFrame({
#     'Change': np.repeat([0, 1]),  
#     'success_list': success_list,  
#     'time_list': time_list,
#     'total_poses_list': total_poses_list,
#     'executed_pose_list': executed_pose_list
# })

# # Create a formula specifying the model
# formula = 'success_list + time_list + total_poses_list + executed_pose_list ~ C(Change)'

# # Fit the repeated measures ANOVA model
# model = ols(formula, data=data).fit()

# # Perform the ANOVA
# anova_table = sm.stats.anova_lm(model, typ=2)

# # Print the ANOVA table
# print(anova_table)
# df = pd.DataFrame({'X': np.repeat(['C', 'U'], 30),
#                    'success_list': success_list})

# #perform three-way ANOVA
# model = ols("""success_list  ~ C(X) + C(Y) +
#                C(X):C(Y)""", data=df).fit()

# print("Success:")
# print(sm.stats.anova_lm(model, typ=2))


# df = pd.DataFrame({'X': np.repeat(['C', 'U'], 60),
#                    'Y': np.tile(np.repeat(['B', 'K'], 30), 2),
#                    'time_list': time_list})

# #perform three-way ANOVA
# model = ols("""time_list  ~ C(X) + C(Y) +
#                C(X):C(Y)""", data=df).fit()
# print("Time:")
# print(sm.stats.anova_lm(model, typ=2))


# df = pd.DataFrame({'X': np.repeat(['C', 'U'], 60),
#                    'Y': np.tile(np.repeat(['B', 'K'], 30), 2),
#                    'total_poses_list': total_poses_list})

# #perform three-way ANOVA
# model = ols("""total_poses_list ~ C(X) + C(Y) +
#                C(X):C(Y)""", data=df).fit()

# print("Total # of poses:")
# print(sm.stats.anova_lm(model, typ=2))


# df = pd.DataFrame({'X': np.repeat(['C', 'U'], 60),
#                    'Y': np.tile(np.repeat(['B', 'K'], 30), 2),
#                    'executed_pose_list': executed_pose_list})

# #perform three-way ANOVA
# model = ols("""executed_pose_list ~ C(X) + C(Y) +
#                C(X):C(Y)""", data=df).fit()

# print("Executed pose #:")
# print(sm.stats.anova_lm(model, typ=2))

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
ax.set_yticklabels(['Bottle, Changed', 'Bottle, Unchanged'])
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
plt.savefig('time_dis_cin.png', dpi=300, bbox_inches='tight')


pose_sublists = [total_poses_list[i:i+30] for i in range(0, len(total_poses_list), 30)]

# Combine the data
data = pose_sublists

# Create the plot
fig, ax = plt.subplots()

# Create box plots
box_plot = ax.boxplot(data, vert=False)

# Add labels and title
ax.set_yticklabels(['Bottle, Changed', 'Bottle, Unchanged'])
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
plt.savefig('total_pose_dis_cin.png', dpi=300, bbox_inches='tight')


ex_sublists = [executed_pose_list[i:i+30] for i in range(0, len(executed_pose_list), 30)]

# Combine the data
data = ex_sublists

# Create the plot
fig, ax = plt.subplots()

# Create box plots
box_plot = ax.boxplot(data, vert=False)

# Add labels and title
ax.set_yticklabels(['Bottle, Changed', 'Bottle, Unchanged'])
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
plt.savefig('ex_pose_dis_cin.png', dpi=300, bbox_inches='tight')


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