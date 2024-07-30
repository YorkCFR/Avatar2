#referenced from https://github.com/aris-ai/Audio-and-text-based-emotion-recognition

from transformers import BertForSequenceClassification, BertTokenizer
import torch
import time

# Path to the saved model
model_path = '/home/sentrybot/Desktop/text_emotion_model/model_text.pt'

# Initialize a new instance of BertForSequenceClassification
model = BertForSequenceClassification.from_pretrained('bert-base-uncased', num_labels=4)  # Adjust num_labels as per your task

model=torch.load(model_path)
print(model)



# Ensure the model is in evaluation mode
model.eval()

# Example single text utterance for testing
#single_text = "i am very angry"

# Tokenize the input text
#tokenizer = BertTokenizer.from_pretrained('bert-base-uncased')
#inputs = tokenizer(single_text, return_tensors='pt')
#input_ids = inputs['input_ids']

label_mapping = {'ang': 0, 'hap': 1, 'neu': 2, 'sad': 3}

#

texts = [
    "I'm feeling really happy today!",
    "This news makes me sad.",
    "I am neutral about this decision.",
    "I'm so angry right now!",
    "Feeling joyous and excited about the upcoming event.",
    "The movie was incredibly touching, it brought tears to my eyes.",
    "No strong feelings about this matter.",
    "His behavior really annoyed me.",
    "She looked so disappointed when she heard the news.",
    "Excited to see my friends after a long time apart."
]

# Initialize BERT tokenizer and model
tokenizer = BertTokenizer.from_pretrained('bert-base-uncased')


# Set the model in evaluation mode
model.eval()

# Perform inference for each text
for text in texts:
    start_time = time.time()  # Start time for measuring processing time
    
    # Tokenize the input text
    inputs = tokenizer(text, return_tensors='pt')
    input_ids = inputs['input_ids']

    # Perform inference
    with torch.no_grad():
        outputs = model(input_ids)
        logits = outputs.logits

        # Get predicted label (assuming softmax activation)
        _, predicted = torch.max(logits, dim=1)
        predicted_label_index = predicted.item()

        # Convert predicted label index to emotion string using label_mapping
        predicted_label_string = next(key for key, value in label_mapping.items() if value == predicted_label_index)

        end_time = time.time()  # End time after processing
    
    # Calculate time taken for inference
    time_taken = end_time - start_time

    # Print results including time taken
    print(f"Text: '{text}'")
    print(f"Predicted Emotion: {predicted_label_string}")
    print(f"Time taken: {time_taken:.4f} seconds")
    print()