import random
from datetime import datetime, timedelta
from occupancy_learning import OccupancyLearner

def generate_training_data(num_runs=150):
    print(f"Generating {num_runs} simulated scans with Day + Time trends...")
    learner = OccupancyLearner()

    spots = []
    for section in [1, 2]:
        for row in ['A', 'B', 'C', 'D']:
            for col in range(40):
                spots.append(f"S{section}-{row}{col}")

    # Start our fake history 30 days ago
    start_date = datetime.now() - timedelta(days=30)

    for run in range(num_runs):
        # Randomize the day and the exact hour of the scan
        random_days_added = random.randint(0, 30)
        random_hour = random.randint(0, 23)
        
        simulated_date = start_date + timedelta(days=random_days_added)
        simulated_date = simulated_date.replace(hour=random_hour, minute=random.randint(0, 59))
        
        day_of_week = simulated_date.weekday()

        # ==========================================
        # THE SMART DAY + TIME LOGIC
        # ==========================================
        if day_of_week < 5:  # WEEKDAYS (Monday - Friday)
            if 8 <= random_hour <= 15:    # Peak School Hours (8 AM - 3 PM)
                base_prob = 0.85
            elif 16 <= random_hour <= 19: # Evening Classes (4 PM - 7 PM)
                base_prob = 0.45
            else:                         # Night time (Ghost town)
                base_prob = 0.05
        else:                # WEEKENDS (Saturday - Sunday)
            if 10 <= random_hour <= 16:   # Weekend Midday (Library studiers)
                base_prob = 0.20
            else:                         # Weekend Night
                base_prob = 0.02

        mock_scan = []
        for label in spots:
            # 1. PERMIT SPOTS (Always occupied during the day)
            if label in ["S1-A0", "S1-A1", "S1-A2"]:
                occupied = True if random_hour > 6 else False
                
            # 2. LOW RISK SPOTS (Consistently blocked off)
            elif label.startswith("S2-C") and int(label.split('C')[1]) > 20:
                occupied = False
                
            # 3. NORMAL SPOTS (Use the time slot math)
            else:
                occupied = random.random() < base_prob

            mock_scan.append({
                "label"    : label,
                "x"        : 0.0, 
                "y"        : 0.0,
                "occupied" : occupied,
                "timestamp": simulated_date.isoformat() 
            })

        learner.record_scan(mock_scan, scan_metadata={"note": "Simulated Day & Time"})

    print("\n[Success] Day & Time Dataset generated!")
    learner.generate_report()

if __name__ == "__main__":
    # Generating 150 runs gives the AI enough data to see the time patterns clearly
    generate_training_data(150)