from django.db import models

# Create your models here.
class Telemetry(models.Model):
    created_at = models.DateTimeField(auto_now_add=True)
    drone_id = models.CharField(max_length=64)
    payload = models.JSONField()  # stores raw feed/telemetry

    def __str__(self):
        return f"{self.drone_id} @ {self.created_at}"