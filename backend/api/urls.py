from django.urls import path
from .views import spaces_api, metrics_api, occupancy_update_api

urlpatterns = [
    path('spaces/', spaces_api),
    path('metrics/', metrics_api),
    path("occupancy/update/", occupancy_update_api),
] 