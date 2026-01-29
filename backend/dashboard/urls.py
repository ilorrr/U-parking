from django.urls import path
from .views import index, health

urlpatterns = [
    path('', index, name='dashboard'),        # /api/
    path('health/', health, name='health'),   # /api/health/
]