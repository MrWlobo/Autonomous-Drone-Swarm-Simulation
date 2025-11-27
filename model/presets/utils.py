from ast import literal_eval
from pathlib import Path
import pandas as pd


def get_delivery_locations(city: str, n: int, grid_width: int, grid_height: int):
    DATA_PATH = Path(__file__).parent.parent.parent / "evaluation/validation/insights/delivery_points_relative.csv"
    
    df = pd.read_csv(DATA_PATH, converters={'relative_pos': literal_eval})
    df = df.sample(frac=1).reset_index(drop=True)
    
    df = df[df["city"] == city].iloc[:n, :][["relative_pos"]]
    
    df[['x', 'y']] = pd.DataFrame(df['relative_pos'].tolist(), index=df.index)
    
    df['x'] = (df['x'] * grid_width).round().clip(lower=0, upper=grid_width).astype(int)
    df['y'] = (df['y'] * grid_height).round().clip(lower=0, upper=grid_height).astype(int)
    
    df = df[['x', 'y']]
    df = df.drop_duplicates()
    
    result = list(zip(df['x'], df['y']))
    
    print(result)
    
    return result