# my_xarm_app/llm/test_ollama.py
import json
from my_xarm6_app.llm.ollama_client import OllamaClient


def main():
    client = OllamaClient(model="llama3")

    system_prompt = """
    You are a robotics assistant that outputs ONLY JSON.
    Given a user command, respond with a JSON containing a target pose.

    Format:
    {
      "frame": "link_base",
      "position": {"x": <float>, "y": <float>, "z": <float>},
      "orientation_rpy": {"roll": <float>, "pitch": <float>, "yaw": <float>}
    }
    Units: meters and radians.
    """

    user_prompt = "Move the TCP 20 cm forward and 10 cm up, orientation roll=0, pitch=90 deg, yaw=0."

    print("🧠 Sending request to Ollama...")
    reply = client.chat(system_prompt, user_prompt)

    print("\n💬 Raw reply from LLM:")
    print(reply)

    try:
        pose = json.loads(reply)
        print("\n✅ Parsed JSON:", pose)
    except json.JSONDecodeError as e:
        print("\n❌ Failed to parse JSON:", e)


if __name__ == "__main__":
    main()