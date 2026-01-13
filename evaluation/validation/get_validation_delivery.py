import numpy as np
import pandas as pd
from pathlib import Path

def calculate_avg_time_per_meter_by_aoi(file_path: Path):
    df = pd.read_csv(file_path)
    
    AOI_IDS = {
        "Chongqing": 38774,
        "Hangzhou": 35806,
        "Shanghai": 56909,
        "Yantai": 31702,
    }
    
    df = df[df["aoi_id"].isin(AOI_IDS.values())]
    
    placeholder_year = "2025-" 
    df['accept_time'] = pd.to_datetime(placeholder_year + df['accept_time'].astype(str))
    df['delivery_time'] = pd.to_datetime(placeholder_year + df['delivery_time'].astype(str))
    
    df['duration_seconds'] = (df['delivery_time'] - df['accept_time']).dt.total_seconds()
    
    # Distance Calculation (Haversine)
    R = 6371000
    lat1 = np.radians(df['accept_gps_lat'])
    lon1 = np.radians(df['accept_gps_lng'])
    lat2 = np.radians(df['delivery_gps_lat'])
    lon2 = np.radians(df['delivery_gps_lng'])
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = np.sin(dlat / 2.0)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    
    df['distance_meters'] = R * c
    
    df = df[df['duration_seconds'] > 0]
    df['speed_mps'] = df['distance_meters'] / df['duration_seconds']
    
    df = df[df['speed_mps'].between(0.1, 35)]

    def filter_middle_90(group):
        d_low = group['distance_meters'].quantile(0.05)
        d_high = group['distance_meters'].quantile(0.95)

        t_low = group['duration_seconds'].quantile(0.05)
        t_high = group['duration_seconds'].quantile(0.95)
        
        mask = (
            (group['distance_meters'].between(d_low, d_high)) & 
            (group['duration_seconds'].between(t_low, t_high))
        )
        return group[mask]

    df = df.groupby('aoi_id').apply(filter_middle_90).reset_index(drop=True)

    valid_data = df.copy()

    groups = valid_data.groupby('aoi_id')
    
    weighted_means = groups['duration_seconds'].sum() / groups['distance_meters'].sum()

    id_to_city = {v: k for k, v in AOI_IDS.items()}
    weighted_means = weighted_means.rename(index=id_to_city)
    
    return weighted_means


if __name__ == "__main__":
    DATA_PATH = Path(__file__).parent / "data" / "LaDe-D.csv"
    RESULTS_PATH = Path(__file__).parent / "insights"
    
    grouped_means = calculate_avg_time_per_meter_by_aoi(DATA_PATH)
    grouped_means.to_csv(RESULTS_PATH / "delivery_times.csv", index_label="city", header=["avg_time_per_meter"])