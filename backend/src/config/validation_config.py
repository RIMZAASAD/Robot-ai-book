from pydantic import BaseSettings
from typing import Optional


class ValidationConfig(BaseSettings):
    """
    Validation-specific configuration settings loaded from environment variables
    """
    # Validation Parameters
    validation_top_k: int = 5
    validation_threshold: float = 0.7
    validation_timeout: int = 30
    validation_batch_size: int = 10

    # Performance Monitoring
    max_retrieval_latency: float = 2.0  # seconds
    min_precision_score: float = 0.8    # 80%
    min_metadata_accuracy: float = 0.95  # 95%
    min_determinism_score: float = 0.9  # 90%

    # Validation Report Settings
    validation_report_retention_days: int = 30

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


# Global validation config instance
validation_config = ValidationConfig()