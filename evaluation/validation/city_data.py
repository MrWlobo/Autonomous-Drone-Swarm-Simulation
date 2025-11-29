import pandas as pd
from pathlib import Path
import requests


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
            response = requests.post(url, json=payload)
            if response.status_code == 200:
                results.extend(response.json()['results'])
            else:
                print(f"Error on batch starting at index {i}")
        except Exception as e:
            print(f"Request failed for batch {i}: {e}")
            
    return results
        

if __name__ == "__main__":
    get_city_areas()
    bounding_boxes_d, bounding_boxes_p = get_aoi_areas()
    get_aoi_targets(bounding_boxes_d.copy())