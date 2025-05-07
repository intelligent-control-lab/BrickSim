import numpy as np
import matplotlib.pyplot as plt

# Load data
perf_none = np.load('perf_none.npy')
perf_simple = np.load('perf_simple.npy')
perf_numpy = np.load('perf_numpy_vectorized.npy')
perf_torch = np.load('perf_torch_vectorized.npy')

# Extract columns
x_none, y_none, yerr_none = perf_none.T
x_simple, y_simple, yerr_simple = perf_simple.T
x_numpy, y_numpy, yerr_numpy = perf_numpy.T
x_torch, y_torch, yerr_torch = perf_torch.T

# Create figure sized for A4 width
plt.figure(figsize=(8, 3))

# Plot in requested order with fixed colors

# Simple
line_simple, = plt.plot(x_simple, y_simple, label='Simple', color='orange')
plt.fill_between(x_simple, y_simple - yerr_simple, y_simple + yerr_simple, alpha=0.3, color='orange')
# Mark early-stop
plt.plot(x_simple[-1], y_simple[-1], marker='x', color='red', linestyle='none')

# NumPy
line_numpy, = plt.plot(x_numpy, y_numpy, label='NumPy', color='blue')
plt.fill_between(x_numpy, y_numpy - yerr_numpy, y_numpy + yerr_numpy, alpha=0.3, color='blue')

# PyTorch (CPU)
line_torch, = plt.plot(x_torch, y_torch, label='PyTorch (CPU)', color='red')
plt.fill_between(x_torch, y_torch - yerr_torch, y_torch + yerr_torch, alpha=0.3, color='red')

# Assembly Disabled
line_none, = plt.plot(x_none, y_none, label='Assembly Disabled', color='grey')
plt.fill_between(x_none, y_none - yerr_none, y_none + yerr_none, alpha=0.3, color='grey')

# Axis labels
plt.xlabel('Number of Bricks')
plt.ylabel('Average Step Time (ms)')

# Legend
plt.legend(loc='upper right')

plt.grid(True, which='both', linestyle='--', linewidth=0.5)

plt.tight_layout()

# Save to PDF
plt.savefig('perf.pdf', bbox_inches='tight')

# Show plot
plt.show()

print(perf_torch[-1,1] - perf_none[-1,1])
