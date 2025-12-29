try:
    import cohere
    _HAS_COHERE = True
except Exception:
    cohere = None
    _HAS_COHERE = False

from ..config.settings import settings
from .base_service import BaseService
import logging

class LLMService(BaseService):
    """
    Service for generating answers using Cohere Chat API based on retrieved context
    """

    def __init__(self):
        super().__init__()
        if _HAS_COHERE:
            self.co = cohere.Client(api_key=settings.cohere_api_key)
            self.model = settings.cohere_chat_model
        else:
            self.co = None
            self.model = None
            self.logger.warning("Cohere SDK not available; LLMService will use a simple fallback response for local development.")

    async def generate_answer(self, query: str, context_chunks: list) -> str:
        """
        Generate an answer for the query using the provided context chunks
        """
        if not self.co:
            # Simple fallback when Cohere is unavailable: return a short summary of the provided context
            if not context_chunks:
                return "I'm sorry, I couldn't find any relevant information in the textbook to answer your question."
            snippets = [c.get('content', '') for c in context_chunks[:3]]
            summary = "\n\n--- Context snippets (local fallback) ---\n\n" + "\n\n".join(snippets)
            return f"(Cohere unavailable in this environment) Here are the top context snippets I could find: {summary}"

        try:
            if not context_chunks:
                return "I'm sorry, I couldn't find any relevant information in the textbook to answer your question."

            # Construct context string
            context_text = "\n\n".join([f"Source: {c.get('source_url', 'Unknown')}\n{c.get('content', '')}" for c in context_chunks])

            # Construct prompt
            system_message = (
                "You are an expert AI Assistant specialized in Physical AI and Humanoid Robotics. "
                "Use the following retrieved context from the textbook to answer the user's question. "
                "If the answer is not in the context, say that you don't know based on the textbook, "
                "but try to be as helpful as possible using the available information. "
                "Keep your answer concise, accurate, and professional."
            )

            # Use Cohere chat
            response = self.co.chat(
                message=query,
                model=self.model,
                preamble=system_message,
                documents=[{"title": f"Chunk {i}", "snippet": c.get('content', '')} for i, c in enumerate(context_chunks)]
            )

            return response.text

        except Exception as e:
            self.logger.error(f"Error generating answer: {str(e)}")
            return f"Error: {str(e)}"

    async def execute(self, query: str, context_chunks: list) -> str:
        """
        Implementation of the abstract execute method
        """
        return await self.generate_answer(query, context_chunks)
