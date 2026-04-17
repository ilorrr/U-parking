import json
import pandas as pd
import matplotlib.pyplot as plt

def generate_daily_graphs(json_path="occupancy_learning.json"):
    print("Loading database...")
    with open(json_path, 'r') as f:
        data = json.load(f)

    runs = data.get('runs', [])
    if not runs:
        print("No data found!")
        return

    # 1. Convert the JSON history into a Pandas DataFrame for easy math
    df = pd.DataFrame([{
        'run_id': r['run_id'],
        'timestamp': pd.to_datetime(r['timestamp']),
        'hour': r['hour'],
        'occupied': r['occupied'],
        'total': r['total']
    } for r in runs])

    print(f"Loaded {len(df)} historical scans.")

    # 2. Plot: Average Occupancy by Hour of Day
    # Group the data by hour and calculate the mean (average) occupied spots
    hourly_avg = df.groupby('hour')['occupied'].mean().reset_index()
    
    plt.figure(figsize=(10, 5))
    plt.bar(hourly_avg['hour'], hourly_avg['occupied'], color='#1f77b4', edgecolor='black')
    plt.title('Average Parking Lot Occupancy by Hour', fontsize=14)
    plt.xlabel('Hour of Day (24-Hour Format)', fontsize=12)
    plt.ylabel('Average Occupied Spots', fontsize=12)
    plt.xticks(range(0, 24)) # Force the X-axis to show all 24 hours
    plt.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig('hourly_average.png')
    print("Saved -> hourly_average.png")

    # 3. Plot: Occupancy Trend Over Time
    plt.figure(figsize=(12, 6))
    plt.plot(df.index, df['occupied'], marker='o', linestyle='-', color='#2ca02c')
    plt.title('Total Parking Lot Occupancy (Historical Trend)', fontsize=14)
    plt.xlabel('Scan Run Number', fontsize=12)
    plt.ylabel('Number of Occupied Spots', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('occupancy_trend.png')
    print("Saved -> occupancy_trend.png")

if __name__ == "__main__":
    generate_daily_graphs()