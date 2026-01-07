import requests

class OllamaClient:
    def __init__(self, model="llama3", base_url="http://localhost:11434"):
        self.model = model
        self.base_url = base_url.rstrip("/")

    def chat(self, system_prompt: str, user_prompt: str) -> str:
        """
        Send a simple chat completion request to the local Ollama server.
        Returns the assistant's response as plain text.
        """
        url = f"{self.base_url}/api/chat"
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            "stream": False
        }

        response = requests.post(url, json=payload)
        response.raise_for_status()
        data = response.json()
        return data["message"]["content"]