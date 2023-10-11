import itertools
import numpy as np

save_dir = '/Users/andresmorfin/skyblue/am/bluesky/scenario/FLOW/'

# Sample lists
cluster_distances = [100, 500, 1000, 2000, 2500, 3000]
traffic_numbers = [50, 100, 150, 250, 300, 350, 400, 450, 500, 550, 600]
random_seeds = np.random.randint(1, 1000, size=5)

combinations = itertools.product(traffic_numbers, cluster_distances, random_seeds)

# Iterate through the combinations and create files with lines containing each value
for combination in combinations:
    # Extract values from the combination
    trafffic_number, cluster_distance, seed = combination
    # Create file name by combining items from the lists and the seed
    file_name = f'{trafffic_number}_{cluster_distance}_{seed}.scn'
    # Open the file and write lines with each value and the seed
    with open(save_dir + file_name, 'w') as file:
        
        line_1 = '00:00:00>streetsenable'
        line_2 = '00:00:00>STOPSIMT 36000'
        line_3 = '00:00:00>STARTLOG'
        line_4 = f'00:00:00>trafficnumber {trafffic_number}'
        line_5 = f'00:00:00>SETCLUSTERDISTANCE {cluster_distance}'
        line_6 = f'00:00:00>SEED {seed}'

        lines = '\n'.join([line_1, line_2, line_3, line_4, line_5, line_6])

        # Write lines with each value and the seed to the file
        file.write(lines)

