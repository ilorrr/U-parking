from django.urls import path
from .views import spaces_api

urlpatterns = [
    path('spaces/', spaces_api),
]