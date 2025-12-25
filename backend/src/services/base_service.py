from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
import logging


class BaseService(ABC):
    """
    Base service class that provides common functionality for all services
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

    def handle_error(self, error: Exception, context: str = "") -> Dict[str, Any]:
        """
        Standardized error handling for services
        """
        error_msg = f"Error in {context}: {str(error)}" if context else f"Error: {str(error)}"
        self.logger.error(error_msg)

        return {
            "error": str(error),
            "context": context,
            "success": False
        }

    @abstractmethod
    async def execute(self, *args, **kwargs) -> Any:
        """
        Execute the service operation - to be implemented by subclasses
        """
        pass