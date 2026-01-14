from pathlib import Path
import pandas as pd
import kagglehub
import os

def generate_uav_report():
    try:
        path = kagglehub.dataset_download("ziya07/uav-coordination-dataset")
    except Exception as e:
        return

    csv_files = [f for f in os.listdir(path) if f.endswith('.csv')]
    if not csv_files:
        print("No CSV file found in the dataset folder.")
        return
    
    file_path = os.path.join(path, csv_files[0])
    print(f"Processing file: {csv_files[0]}")

    df = pd.read_csv(file_path)

    col_map = {
        'Altitude': 'Altitude',
        'Task_Success': 'Task_Success',
        'Task_Type': 'Task_Type',
    }
    
    missing_cols = [c for c in col_map.keys() if c not in df.columns]
    if missing_cols:
        return

    global_stats = {
        'Metric_Scope': 'Global_Dataset_Average',
        'Altitude_Mean': df['Altitude'].mean(),
        'Altitude_Std': df['Altitude'].std(),
        'Altitude_Max': df['Altitude'].max(),
        'Altitude_Min': df['Altitude'].min(),
        'Task_Success_Rate': df['Task_Success'].mean(),
        'Sample_Count': len(df)
    }

    if 'Task_Type' in df.columns:
        grouped_stats = df.groupby('Task_Type').agg({
            'Altitude': ['mean', 'std', 'min', 'max'],
            'Task_Success': 'mean',
            'UAV_ID': 'count'
        }).reset_index()

        grouped_stats.columns = ['Task_Type', 'Altitude_Mean', 'Altitude_Std', 'Altitude_Min', 'Altitude_Max', 'Task_Success_Rate', 'Sample_Count']
        grouped_stats['Metric_Scope'] = grouped_stats['Task_Type']
        grouped_stats = grouped_stats.drop(columns=['Task_Type'])
    else:
        grouped_stats = pd.DataFrame()

    global_df = pd.DataFrame([global_stats])
    final_report = pd.concat([global_df, grouped_stats], ignore_index=True)

    cols = ['Metric_Scope', 'Task_Success_Rate', 'Altitude_Mean', 'Altitude_Std', 'Altitude_Min', 'Altitude_Max', 'Sample_Count']
    final_cols = [c for c in cols if c in final_report.columns]
    final_report = final_report[final_cols]

    output_filename = Path(__file__).parent / "insights" / "task_stats.csv"
    final_report.to_csv(output_filename, index=False)
    
    print("\nPreview of Results:")
    print(final_report.to_string())

if __name__ == "__main__":
    generate_uav_report()