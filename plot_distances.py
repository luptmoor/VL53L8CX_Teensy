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
    blueprint.spawn("distance_data")

def main():
    parser = argparse.ArgumentParser(description="Plot distances from a CSV file in rerun.")
    parser.add_argument("--csv_file", type=str, default="data/tof_data_03.csv", help="Path to the CSV file containing distance data.")
    args = parser.parse_args()

    # Initialize rerun
    init_rerun()

    data = pd.read_csv(args.csv_file)
    print(data.head())

    # Identify all columns ending with '_Distance_mm' and sort them for consistent ordering
    distance_cols = sorted([col for col in data.columns if col.endswith('_Distance_mm')])

    # For each row, extract the distance values and reshape into 8x8 numpy arrays
    for index, row in data.iterrows():
        distances = row[distance_cols].to_numpy(dtype=np.float32)
        grid = distances.reshape((8, 8))
        rr.log("dist", rr.DepthImage(grid, meter=1000.0))
        






if __name__ == "__main__":
    main()
