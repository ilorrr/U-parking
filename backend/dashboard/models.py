from django.db import models

# parking model draft code

class ParkingSpot(models.Model):
    spot_id = models.CharField(max_length=20, unique=True)
    is_occupied = models.BooleanField(default=False)
    last_updated = models.DateTimeField(auto_now=True)

    def __str__(self):
        return f"Spot {self.spot_id} - {'Occupied' if self.is_occupied else 'Free'}"
    
    
