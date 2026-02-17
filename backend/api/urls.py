from django.urls import path
from .views import spaces_api, metrics_api

urlpatterns = [
    path('spaces/', spaces_api),
    path('metrics/', metrics_api),
]