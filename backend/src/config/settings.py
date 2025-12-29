from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables
    """
    # Cohere Configuration
    cohere_api_key: str
    cohere_model: str = "embed-english-v3.0"
    cohere_chat_model: str = "command-r7b-12-2024"
    
    # OpenRouter Configuration
    openrouter_api_key: Optional[str] = None
    openrouter_model: str = "mistralai/devstral-2512:free"
    openrouter_base_url: str = "https://openrouter.ai/api/v1"

    # Qdrant Configuration
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_host: str = "localhost"
    qdrant_port: int = 6333

    # Application Configuration
    host: str = "0.0.0.0"
    port: int = 8000

    # Text Processing Configuration
    chunk_size: int = 512
    chunk_overlap: int = 102  # 20% of chunk_size (512 * 0.2 = 102.4, rounded to 102)

    # URL Processing Configuration
    url_timeout: int = 30
    max_retries: int = 3

    # Collection name for Qdrant
    collection_name: str = "textbook_content"

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore"
    )


# Global settings instance
settings = Settings()