import os
import pandas as pd
from pathlib import Path
import requests
import tqdm


if __name__ == "__main__":
    SAVE_PATH = Path(__file__).parent / "data"
    os.makedirs(SAVE_PATH, exist_ok=True)

    # get Lade-D (Delivery)
    splits = {'delivery_cq': 'data/delivery_cq-00000-of-00001-465887add76aeabc.parquet', 'delivery_hz': 'data/delivery_hz-00000-of-00001-8090c86f64781f71.parquet', 'delivery_jl': 'data/delivery_jl-00000-of-00001-a4fbefe3c368583c.parquet', 'delivery_sh': 'data/delivery_sh-00000-of-00001-ad9a4b1d79823540.parquet', 'delivery_yt': 'data/delivery_yt-00000-of-00001-cc85c1fcb1d10955.parquet'}
    d = pd.DataFrame()
    for _, filename in splits.items():
        d = pd.concat([d, pd.read_parquet("hf://datasets/Cainiao-AI/LaDe-D/" + filename)])

    d.to_csv(SAVE_PATH / "LaDe-D.csv")


    # get LaDe-P (Pickup)
    splits = {'pickup_cq': 'data/pickup_cq-00000-of-00001-a172031e5392f9d3.parquet', 'pickup_hz': 'data/pickup_hz-00000-of-00001-2641abebfe50648a.parquet', 'pickup_jl': 'data/pickup_jl-00000-of-00001-9b430a56a935f284.parquet', 'pickup_sh': 'data/pickup_sh-00000-of-00001-79fabe8088e723a2.parquet', 'pickup_yt': 'data/pickup_yt-00000-of-00001-6d21a4dccd28ee03.parquet'}
    p = pd.DataFrame()
    for _, filename in splits.items():
        p = pd.concat([p, pd.read_parquet("hf://datasets/Cainiao-AI/LaDe-P/" + filename)])
    p.to_csv(SAVE_PATH / "LaDe-P.csv")
    
    
    BUILDING_DATA_URLS = {
        "Hangzhou": "https://zenodo.org/records/7923866/files/CNBH10m_X121Y31.tif?download=1",
        "Shanghai": "https://zenodo.org/records/7923866/files/CNBH10m_X121Y31.tif?download=1",
        "Yantai": "https://zenodo.org/records/7923866/files/CNBH10m_X121Y37.tif?download=1",
        "Chongqing": "https://zenodo.org/records/7923866/files/CNBH10m_X107Y29.tif?download=1",
    }
    
    for city, url in BUILDING_DATA_URLS.items():
        with requests.get(url, stream=True) as response:
            response.raise_for_status()
            
            total_size = int(response.headers.get('content-length', 0))
            block_size = 8192
            
            path = SAVE_PATH / f"{city}.tif"
            
            with tqdm.tqdm(total=total_size, unit='iB', unit_scale=True) as progress_bar:
                with open(path, 'wb') as file:
                    for chunk in response.iter_content(chunk_size=block_size):
                        if chunk:
                            progress_bar.update(len(chunk))
                            file.write(chunk)