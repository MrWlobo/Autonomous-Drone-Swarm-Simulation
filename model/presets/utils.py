from ast import literal_eval
import ast
import json
from pathlib import Path
import pandas as pd


def load_elevation_grid(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    parsed_data = {ast.literal_eval(k): v for k, v in data.items()}
    
    return parsed_data


def get_delivery_locations(city: str, n: int, grid_width: int, grid_height: int):
    DATA_PATH = Path(__file__).parent.parent.parent / "evaluation/validation/insights/delivery_points_relative.csv"
    
    df = pd.read_csv(DATA_PATH, converters={'relative_pos': literal_eval})
    df = df.sample(frac=1).reset_index(drop=True)
    
    df = df[df["city"] == city].iloc[:n, :][["relative_pos"]]
    
    df[['x', 'y']] = pd.DataFrame(df['relative_pos'].tolist(), index=df.index)
    
    df['x'] = (df['x'] * (grid_width - 1) + 1).round().clip(lower=1, upper=grid_width).astype(int)
    df['y'] = (df['y'] * (grid_height - 1) + 1).round().clip(lower=1, upper=grid_height).astype(int)
    
    df = df[['x', 'y']]
    df = df.drop_duplicates()
    
    result = list(zip(df['x'], df['y']))
    
    return result