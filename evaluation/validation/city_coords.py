import pandas as pd
from pathlib import Path


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
        min_lat=('lat', 'min')
    ).reset_index()
    
    bounding_boxes.to_csv(Path(__file__).parent / "insights" / "city_areas.csv", index=False)



if __name__ == "__main__":
    get_city_areas()