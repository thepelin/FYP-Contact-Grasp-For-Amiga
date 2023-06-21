import matplotlib.pyplot as plt
import numpy as np

# Example data
data1 = [1, 2, 3, 4, 5]
data2 = [2, 4, 6, 8, 10]
data3 = [3, 6, 9, 12, 15, 16, 17]

# Combine the data
data = [data1, data2, data3]

# Create the plot
fig, ax = plt.subplots()

# Create box plots
box_plot = ax.boxplot(data, vert=False)

# Add labels and title
ax.set_xticklabels(['Data 1', 'Data 2', 'Data 3'])
ax.set_xlabel('Dataset')
ax.set_ylabel('Value')
ax.set_title('Distribution of Data')

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
plt.show()

