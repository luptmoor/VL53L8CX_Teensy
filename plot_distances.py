import argparse

import numpy as np
import pandas as pd
import rerun as rr
import rerun.blueprint as rrb


def init_rerun():
    # Define a blueprint for the distance data
    # rr.set_time("sensor_time", timestamp=0)
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(
                    name="Distance Bottom",
                    origin="dist_bottom",
                ),
                rrb.Spatial2DView(
                    name="Distance Forward",
                    origin="dist_forward",
                ),
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="Accelerometer",
                    origin="/accel",
                    time_ranges=
                    [
                        rrb.VisibleTimeRange(
                            timeline="sensor_time",
                            start=rrb.TimeRangeBoundary.cursor_relative(seconds=-10),
                            end=rrb.TimeRangeBoundary.cursor_relative(seconds=2),
                        )
                    ],
                ),
                rrb.TimeSeriesView(
                    name="Gyroscope",
                    origin="/gyro",
                    time_ranges=
                    [
                        rrb.VisibleTimeRange(
                            timeline="sensor_time",
                            start=rrb.TimeRangeBoundary.cursor_relative(seconds=-10),
                            end=rrb.TimeRangeBoundary.cursor_relative(seconds=2),
                        )
                    ],
                ),
                rrb.TimeSeriesView(
                    name="Attitude",
                    origin="/att",
                    time_ranges=
                    [
                        rrb.VisibleTimeRange(
                            timeline="sensor_time",
                            start=rrb.TimeRangeBoundary.cursor_relative(seconds=-6),
                            end=rrb.TimeRangeBoundary.cursor_relative(seconds=2),
                        )
                    ],
                ),
            ),
            column_shares=[1, 2]        
        ),
    )
    rr.init("tof_viz", spawn=True)
    # rr.set_blueprint(blueprint)
    rr.send_blueprint(blueprint)

def main():
    parser = argparse.ArgumentParser(description="Plot distances from a CSV file in rerun.")
    parser.add_argument("--csv_file", type=str, default="data/tof_data_30.csv", help="Path to the CSV file containing distance data.")
    args = parser.parse_args()

    # Initialize rerun
    init_rerun()

    data = pd.read_csv(args.csv_file)

    # Identify all columns ending with '_Distance_mm' and sort them for consistent ordering
    # Extract the distance columns and sort them by zone number
    distance_cols = [col for col in data.columns if col.endswith('_Distance_mm')]
    distance_cols = sorted(distance_cols, key=lambda col: int(col.split('Zone')[1].split('_')[0]))

    status_cols = [col for col in data.columns if col.endswith('_Status')]
    status_cols = sorted(status_cols, key=lambda col: int(col.split('Zone')[1].split('_')[0]))

    res = (8, 8)
    if data.loc[:, distance_cols[16:]].isna().all().all():
        distance_cols = distance_cols[:16]
        status_cols = status_cols[:16]
        res = (4, 4)

    # For each row, extract the distance values and reshape into 8x8 numpy arrays
    for index, row in data.iterrows():
        distances = row[distance_cols].to_numpy(dtype=np.float32)
        grid = distances.reshape(res)
        grid = np.flipud(grid)  # Flip the grid vertically to match the expected orientation
        grid = np.fliplr(grid) # Flip the grid horizontally to match the expected orientation
        grid[np.isnan(grid)] = 4000

        status = row[status_cols].to_numpy(dtype=np.float32).reshape(res)
        invalid_mask = ~np.isin(status, [5, 9, 10, 255])
        grid[invalid_mask] = np.nan
        inf_mask = np.isin(status, [255])
        grid[inf_mask] = np.inf
        grid[grid < 0] = np.nan  # Set negative distances to NaN

        rr.set_time("sensor_time", timestamp=row["Time"]/1000)
        if row["Sensor"] == 0:
            # print(status[status != 5])
            grid = np.rot90(grid, k=1)  # Rotate the grid 90 degrees clockwise for bottom sensor
            rr.log("dist_bottom", rr.DepthImage(grid, meter=1000.0, colormap="viridis", depth_range=(400, 1200)))
        elif row["Sensor"] == 1:
            # print(status[status != 5])
            rr.log("dist_forward", rr.DepthImage(grid, meter=1000.0, colormap="viridis", depth_range=(0, 3000)))
        elif row["Sensor"] == 2:
            rr.log("accel/x", rr.Scalars(row["AccX"]))
            rr.log("accel/y", rr.Scalars(row["AccY"]))
            rr.log("accel/z", rr.Scalars(row["AccZ"]))
            rr.log("gyro/x", rr.Scalars(row["Roll_Gyro"]))
            rr.log("gyro/y", rr.Scalars(row["Pitch_Gyro"]))
            rr.log("gyro/z", rr.Scalars(row["Yaw_Gyro"]))
            rr.log("att/roll", rr.Scalars(row["Roll"]))
            rr.log("att/rollt", rr.Scalars(row["RollT"]))
            rr.log("att/pitch", rr.Scalars(row["Pitch"]))
            rr.log("att/pitcht", rr.Scalars(row["PitchT"]))


if __name__ == "__main__":
    main()
