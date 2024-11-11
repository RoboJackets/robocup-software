import re
import matplotlib.pyplot as plt
import numpy as np

def extract_times(file_path):
    """
    Extracts time values from a file where each line is in the format:
    'CreatePath::intermediate() Time: <number> <unit>'
    Supports both 'ns' and 's' units.
    """
    times = []
    units = []  # List to keep track of units (ns or s)
    
    with open(file_path, 'r') as f:
        for line in f:
            # Use regex to extract the time value and its unit (ns or s)
            match = re.search(r'Time:\s*([\d\.]+)\s*(ns|s)', line)
            if match:
                time_value = float(match.group(1))
                unit = match.group(2)
                
                # Store the time and unit in their respective lists
                times.append(time_value)
                units.append(unit)
    
    return times, units

def calculate_stats(times):
    """
    Calculates and returns basic statistics for the time values:
    mean, median, standard deviation, minimum, and maximum.
    """
    mean = np.mean(times)
    median = np.median(times)
    std_dev = np.std(times)
    min_val = np.min(times)
    max_val = np.max(times)
    
    return mean, median, std_dev, min_val, max_val

def plot_distribution(times, units, title, subplot_idx):
    """
    Plots the distribution of the times using a histogram.
    """
    plt.subplot(1, 2, subplot_idx)  # Create a subplot (1 row, 2 columns)
    
    # Increase the number of bins for finer granularity
    bins = 50  # Increase bins to get more granularity in the lower end of the distribution
    
    # Plot histogram
    density, bins, _ = plt.hist(times, bins=bins, density=True, color='blue', alpha=0.7, edgecolor='black')
    
    # Determine the unit for x-axis label
    if units[0] == 'ns':
        x_label = "Time (ns)"
    elif units[0] == 's':
        x_label = "Time (s)"
    
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel("Density")
    plt.legend()

def print_stats(title, times, units):
    """
    Prints the statistics for a given dataset.
    """
    mean, median, std_dev, min_val, max_val = calculate_stats(times)
    
    # Display the unit (ns or s) for reference
    unit_str = "ns" if units[0] == 'ns' else "s"
    
    print(f"Statistics for {title}:")
    print(f"  Mean: {mean:.2f} {unit_str}")
    print(f"  Median: {median:.2f} {unit_str}")
    print(f"  Standard Deviation: {std_dev:.2f} {unit_str}")
    print(f"  Min: {min_val} {unit_str}")
    print(f"  Max: {max_val} {unit_str}")
    print("-" * 50)

def main():
    # File paths
    file_path_intermediate = 'intermediate.out'
    file_path_rrt = 'rrt.out'
    
    # Extract times and units from both files
    times_intermediate, units_intermediate = extract_times(file_path_intermediate)
    times_rrt, units_rrt = extract_times(file_path_rrt)
    
    # Create a figure for the two plots
    plt.figure(figsize=(14, 6))
    
    # Plot and calculate statistics for intermediate.out
    if times_intermediate:
        plot_distribution(times_intermediate, units_intermediate, "Distribution of intermediate.out", 1)
        print_stats("intermediate.out", times_intermediate, units_intermediate)
    else:
        print(f"No valid times found in {file_path_intermediate}.")
    
    # Plot and calculate statistics for rrt.out
    if times_rrt:
        plot_distribution(times_rrt, units_rrt, "Distribution of rrt.out", 2)
        print_stats("rrt.out", times_rrt, units_rrt)
    else:
        print(f"No valid times found in {file_path_rrt}.")
    
    # Show the plots
    plt.tight_layout()  # Adjust layout to prevent overlap
    plt.show()

if __name__ == '__main__':
    main()
