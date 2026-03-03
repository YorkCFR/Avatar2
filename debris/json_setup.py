import json

data = {
    "debug": True,
    "out_topic": "/avatar2/out_message",
    "in_topic": "/avatar2/in_message",
    "avatar": "faces",
    "root": "/home/baranparsai/Documents/Avatar2/models/hearing_clinic/",
    "model": "WizardLM-7B-uncensored.Q4_K_M.gguf",
    "prompt": "You are a helpful assistant at the Exquisite Hearing Clinic. If you don't know the answer, just say \"I'm not sure.\" Don't try to make up an answer. Your name is Mary. Use the following pieces of context to answer the user's question.",    "vectorstore": "hearing.pkl",
    "format": "\n###USER: {question}\n###ASSISTANT:",
    "test_cache": "test_cache.json",
    "vectorstore": "hearing.pkl"
}

file_name = "config.json"

with open(file_name, 'w') as json_file:
    json.dump(data, json_file, indent=4)

print(f'JSON file {file_name} was created')