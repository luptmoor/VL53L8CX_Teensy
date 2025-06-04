import argparse

import numpy as np
import pandas as pd
import rerun as rr
import rerun.blueprint as rrb


def init_rerun():
    # Define a blueprint for the distance data
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(
                name="distance_data",
                origin="dist",
            )
        ),
        # collapse_panels=True,
    )
    rr.init("tof_viz", spawn=True)
    rr.send_blueprint(blueprint)

def main():
    parser = argparse.ArgumentParser(description="Plot distances from a CSV file in rerun.")
    parser.add_argument("--csv_file", type=str, default="data/tof_data_03.csv", help="Path to the CSV file containing distance data.")
    args = parser.parse_args()

    # Initialize rerun
    init_rerun()

    data = pd.read_csv(args.csv_file)

    # Identify all columns ending with '_Distance_mm' and sort them for consistent ordering
    # Extract the distance columns and sort them by zone number
    distance_cols = [col for col in data.columns if col.endswith('_Distance_mm')]
    distance_cols = sorted(distance_cols, key=lambda col: int(col.split('Zone')[1].split('_')[0]))

    res = (8, 8)
    if data.loc[:, distance_cols[16:]].isna().all().all():
        distance_cols = distance_cols[:16]
        res = (4, 4)

    # For each row, extract the distance values and reshape into 8x8 numpy arrays
    for index, row in data.iterrows():
        distances = row[distance_cols].to_numpy(dtype=np.float32)
        grid = distances.reshape(res)
        grid = np.flipud(grid)  # Flip the grid vertically to match the expected orientation
        grid = np.fliplr(grid) # Flip the grid horizontally to match the expected orientation
        rr.set_time("sensor_time", timestamp=row["Time"]/1000)
        rr.log("dist", rr.DepthImage(grid, meter=1000.0, colormap="viridis", depth_range=(0, 3000)))



if __name__ == "__main__":
    main()
