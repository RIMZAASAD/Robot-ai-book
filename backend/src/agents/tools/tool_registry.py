"""
Tool registry for the RAG Agent Construction feature.

This module provides a registry for agent tools that can be used by the agent.
"""

import logging
from typing import Dict, List, Optional, Any
from .retrieval_tool import RetrievalTool


class ToolRegistry:
    """Registry for agent tools"""

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.tools: Dict[str, Any] = {}
        self._initialize_tools()

    def _initialize_tools(self):
        """Initialize the default tools for the agent"""
        # Register the retrieval tool
        retrieval_tool = RetrievalTool()
        self.tools[retrieval_tool.name] = retrieval_tool
        self.logger.info(f"Registered tool: {retrieval_tool.name}")

    def get_tool(self, tool_name: str) -> Optional[Any]:
        """Get a tool by name"""
        return self.tools.get(tool_name)

    def get_tool_names(self) -> List[str]:
        """Get all registered tool names"""
        return list(self.tools.keys())

    def get_tool_descriptions(self) -> List[Dict[str, Any]]:
        """Get descriptions of all registered tools for agent configuration"""
        return [
            {
                "name": name,
                "description": tool.description,
                "schema": tool.input_schema.model_json_schema()
            }
            for name, tool in self.tools.items()
        ]

    def register_tool(self, tool: Any):
        """Register a new tool"""
        self.tools[tool.name] = tool
        self.logger.info(f"Registered new tool: {tool.name}")

    def get_openai_tools(self) -> List[Dict[str, Any]]:
        """Get tools in OpenAI-compatible format"""
        return [tool.to_dict() for tool in self.tools.values()]


# Global instance of the tool registry
tool_registry = ToolRegistry()