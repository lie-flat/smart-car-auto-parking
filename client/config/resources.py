from PIL import ImageFont
from pathlib import Path
from .common import BASE_DIR


RESOURCES_DIR = BASE_DIR/"resources"

CHINESE_FONT_PATH = str(RESOURCES_DIR/"SmileySans-Oblique.ttf")

CHINESE_FONT = ImageFont.truetype(CHINESE_FONT_PATH, 44)

ENVIRONMENT_RESOURCES_DIR = RESOURCES_DIR/"environment"
