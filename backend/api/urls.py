from django.urls import path
from .views import api_view

urlpatterns = [
    path('health/', api_view, name='healt_api'),
]