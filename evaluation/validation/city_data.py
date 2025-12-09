import json
import numpy as np
import rasterio
from rasterio.transform import rowcol
from scipy.ndimage import gaussian_filter, zoom
import pandas as pd
from pathlib import Path
from pyproj import Transformer
import requests

from model.presets.chongqing_38774 import Chongqing38774Preset
from model.presets.shanghai_56909 import Shanghai56909Preset
from model.presets.yantai_31702 import Yantai31702Preset
from model.presets.hangzhou_35806 import Hangzhou35806Preset


def get_city_areas():
    """Calculates the bounding boxes (min/max lattitude and longitude)
    of the cities in the LaDe dataset.
    """
    df_d = pd.read_csv(Path(__file__).parent / "data" / "LaDe-D.csv")
    df_p = pd.read_csv(Path(__file__).parent / "data" / "LaDe-P.csv")
    
    box_d = df_d[['city', 'lng', 'lat']]
    box_p = df_p[['city', 'lng', 'lat']]
    
    box = pd.concat([box_d, box_p])
    
    bounding_boxes = box.groupby('city').agg(
        max_lng=('lng', 'max'),
        min_lng=('lng', 'min'),
        max_lat=('lat', 'max'),
        min_lat=('lat', 'min'),
        num_records=('lng', 'count')
    ).reset_index()
    
    bounding_boxes = bounding_boxes.sort_values(by='num_records', ascending=False)
    
    bounding_boxes.to_csv(Path(__file__).parent / "insights" / "city_areas.csv", index=False)


def get_aoi_areas():
    """Calculates the bounding boxes (min/max lattitude and longitude)
    of all Areas of Interest in LaDe-D and LaDe-P datasets.
    """
    df_d = pd.read_csv(Path(__file__).parent / "data" / "LaDe-D.csv")
    df_p = pd.read_csv(Path(__file__).parent / "data" / "LaDe-P.csv")
    
    box_d = df_d[['city', 'aoi_id', 'lng', 'lat']]
    box_p = df_p[['city', 'aoi_id', 'lng', 'lat']]
    
    
    bounding_boxes_d = box_d.groupby(['city', 'aoi_id']).agg(
        max_lng=('lng', 'max'),
        min_lng=('lng', 'min'),
        max_lat=('lat', 'max'),
        min_lat=('lat', 'min'),
        num_records=('lng', 'count')
    ).reset_index()
    
    bounding_boxes_p = box_p.groupby(['city', 'aoi_id']).agg(
        max_lng=('lng', 'max'),
        min_lng=('lng', 'min'),
        max_lat=('lat', 'max'),
        min_lat=('lat', 'min'),
        num_records=('lng', 'count')
    ).reset_index()
    
    bounding_boxes_d = bounding_boxes_d.sort_values(by='num_records', ascending=False)
    bounding_boxes_p = bounding_boxes_p.sort_values(by='num_records', ascending=False)
    
    bounding_boxes_d.to_csv(Path(__file__).parent / "insights" / "city_aoi_areas_delivery.csv", index=False)
    bounding_boxes_p.to_csv(Path(__file__).parent / "insights" / "city_aoi_areas_pickup.csv", index=False)
    
    return bounding_boxes_d, bounding_boxes_p


def get_aoi_targets(bounding_boxes_d):
    """Calculates the relative position of each delivery point in selected AOIs.
    """
    AOI_IDS = {
        "Chongqing": 38774,
        "Hangzhou": 35806,
        "Shanghai": 56909,
        "Yantai": 31702,
    }
    
    bounding_boxes_d = bounding_boxes_d[bounding_boxes_d["aoi_id"].isin(AOI_IDS.values())]

    df_d = pd.read_csv(Path(__file__).parent / "data" / "LaDe-D.csv")
    
    delivery_targets = df_d[df_d["aoi_id"].isin(AOI_IDS.values())]
    
    d_merged = delivery_targets.merge(bounding_boxes_d, on='city', how='left')
    
    d_merged['rel_west'] = (
        (d_merged['delivery_gps_lng'] - d_merged['min_lng']) / 
        (d_merged['max_lng'] - d_merged['min_lng'])
    )
    
    d_merged['rel_south'] = (
        (d_merged['delivery_gps_lat'] - d_merged['min_lat']) / 
        (d_merged['max_lat'] - d_merged['min_lat'])
    )
    
    d_merged['relative_pos'] = list(zip(d_merged['rel_west'], d_merged['rel_south']))
    
    d_merged = d_merged.iloc[:, 1:][["city", "relative_pos"]]
    
    d_merged.to_csv(Path(__file__).parent / "insights" / "delivery_points_relative.csv", index=False)


def get_elevation_batch(points: list[tuple[float, float]], batch_size: int = 50) -> list[dict[str, float]]:
    """Gets Open-Elevation API data (90m data resolution) for a specified list of (latitude, longitude) points.

    Args:
        points (list[tuple[float, float]]): List of points (latitude, longitude) to get the elevation of.
        batch_size (int, optional): Number of points to process in a batch. Defaults to 50.

    Returns:
        list[dict[str, float]]: A list of dictionaries with the latitude, longitude and elevation of each point.
    """
    url = 'https://api.open-elevation.com/api/v1/lookup'
    results = []
    
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        
        payload = {
            "locations": [{"latitude": lat, "longitude": lon} for lat, lon in batch]
        }
        
        try:
            print(f"Processing batch {i} to {i+len(batch)}...")
            response = requests.post(url, json=payload, timeout=10)
            
            if response.status_code == 200:
                results.extend(response.json()['results'])
            else:
                print(f"Error on batch starting at index {i}: Status {response.status_code}")
                results.extend([{'elevation': 0.0, 'latitude': p[0], 'longitude': p[1]} for p in batch])
                
        except Exception as e:
            print(f"Request failed for batch {i}: {e}")
            results.extend([{'elevation': 0.0, 'latitude': p[0], 'longitude': p[1]} for p in batch])
            
    return results


def get_elevation_for_cities(bounding_boxes_d) -> None:
    SAVE_PATH = Path(__file__).parent.parent.parent / "model/presets/elevation"
    SAVE_PATH.mkdir(parents=True, exist_ok=True)
    
    AOI_IDS = {
        "Chongqing": 38774,
        "Hangzhou": 35806,
        "Shanghai": 56909,
        "Yantai": 31702,
    }
    
    hangzhou_preset = Hangzhou35806Preset()
    yantai_preset = Yantai31702Preset()
    shanghai_preset = Shanghai56909Preset()
    chongqing_preset = Chongqing38774Preset()
    
    GRID_DIMENSIONS = {
        "Chongqing": (chongqing_preset.height, chongqing_preset.width),
        "Hangzhou": (hangzhou_preset.height, hangzhou_preset.width),
        "Shanghai": (shanghai_preset.height, shanghai_preset.width),
        "Yantai": (yantai_preset.height, yantai_preset.width),
    }
    
    bounding_boxes_d = bounding_boxes_d[bounding_boxes_d["aoi_id"].isin(AOI_IDS.values())]
    
    STRIDE = 45
    BLUR_SIGMA = 15

    for city in ("Chongqing", "Hangzhou", "Shanghai", "Yantai"):
        city_box = bounding_boxes_d[bounding_boxes_d["city"] == city]
        
        target_height, target_width = GRID_DIMENSIONS[city]
        
        min_lat = city_box["min_lat"].item()
        max_lat = city_box["max_lat"].item()
        min_lng = city_box["min_lng"].item()
        max_lng = city_box["max_lng"].item()

        # Generate Query Points (Sparse Grid)
        query_points = []
        
        sample_rows_indices = range(0, target_height, STRIDE)
        sample_cols_indices = range(0, target_width, STRIDE)
        
        low_res_h = len(sample_rows_indices)
        low_res_w = len(sample_cols_indices)
        

        for r in sample_rows_indices:
            for c in sample_cols_indices:
                curr_lat = min_lat + (r / target_height) * (max_lat - min_lat)
                curr_lng = min_lng + (c / target_width) * (max_lng - min_lng)
                
                query_points.append((curr_lat, curr_lng))

        raw_results = get_elevation_batch(query_points) 
        
        sampled_elevations = [item['elevation'] for item in raw_results]

        # Create Low-Res Grid
        low_res_grid = np.array(sampled_elevations).reshape((low_res_h, low_res_w))
        
        # Resize to Full Resolution
        zoom_factor_h = target_height / low_res_h
        zoom_factor_w = target_width / low_res_w
        
        elevation_grid_filled = zoom(low_res_grid, (zoom_factor_h, zoom_factor_w), order=0)
        
        elevation_grid_filled = elevation_grid_filled[:target_height, :target_width]

        # Apply Gaussian Blur to the 2D Grid (because the data has 90m granularity)
        print(f"Applying blur (sigma={BLUR_SIGMA})...")
        elevation_grid_blurred = gaussian_filter(elevation_grid_filled, sigma=BLUR_SIGMA)
        
        full_elevations_blurred = elevation_grid_blurred.flatten().tolist()
        
        all_keys = [(c, r) for r in range(target_height) for c in range(target_width)]
        
        elevation_dict = {k: v for k, v in zip(all_keys, full_elevations_blurred)}
        
        # Get building heights
        with rasterio.open(Path(__file__).parent / f"data/{city}.tif") as src:
            height_map = src.read(1)
            
            keys = list(elevation_dict.keys())
            coords_cr = np.array(keys) 
            elevations = np.array(list(elevation_dict.values()))
            
            c_vals = coords_cr[:, 0]
            r_vals = coords_cr[:, 1]

            curr_lats = min_lat + (r_vals / target_height) * (max_lat - min_lat)
            curr_lngs = min_lng + (c_vals / target_width) * (max_lng - min_lng)
            
            transformer = Transformer.from_crs("EPSG:4326", src.crs, always_xy=True)
            xs, ys = transformer.transform(curr_lngs, curr_lats)
            
            rows, cols = rowcol(src.transform, xs, ys)
            rows = np.array(rows)
            cols = np.array(cols)
            
            valid_mask = (
                (rows >= 0) & (rows < height_map.shape[0]) & 
                (cols >= 0) & (cols < height_map.shape[1])
            )
            
            building_heights = np.zeros_like(elevations)
            
            building_heights[valid_mask] = height_map[rows[valid_mask], cols[valid_mask]]
            
            building_heights = np.nan_to_num(building_heights, nan=0.0)
            building_heights[building_heights < 2.0] = 0.0
            
            new_elevations = elevations + building_heights
        
            for i, key in enumerate(keys):
                elevation_dict[key] = new_elevations[i]
                    
            
            # Convert tuple keys to string for JSON compatibility
            curr_result = {str(k): v for k, v in elevation_dict.items()}
        
        output_file = SAVE_PATH / f"{city}_elevation.json"
        with open(output_file, 'w') as f:
            json.dump(curr_result, f, indent=4)


if __name__ == "__main__":
    get_city_areas()
    bounding_boxes_d, bounding_boxes_p = get_aoi_areas()
    get_aoi_targets(bounding_boxes_d.copy())
    get_elevation_for_cities(bounding_boxes_d.copy())
    