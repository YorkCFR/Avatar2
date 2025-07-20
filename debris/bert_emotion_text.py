# %% [markdown]
# ***Install transformer***

# %%
!pip install transformers
!pip install tensorboardx

# %% [markdown]
# ***Import all necessary packages***

# %%
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.backend_bases import RendererBase
import os
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils import data
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from tensorboardX import SummaryWriter
from transformers import BertModel, BertTokenizer
import torch



file_path = "iemocap_transcriptions_labels2.txt"

import pandas as pd
df = pd.read_csv(file_path, sep='\t', header=None, names=['session_ID','Text', 'Label'])

# %% [markdown]
# ***Collect the text and label as list of dict***

# %%
docs=[]
for text,label in zip(df['Text'],df['Label']):
  if(label!=-1):
    docs.append({'Text':text,'Label':label})
print(len(docs))

from sklearn.utils import class_weight
import torch

# Calculate class weights
class_weights = class_weight.compute_class_weight('balanced', classes=np.unique(df['Label']), y=df['Label'])
class_weights = torch.tensor(class_weights, dtype=torch.float)


# %% [markdown]
# ***Create the training and test set***

# %%
import random
random.shuffle(docs)
random.shuffle(docs)
random.shuffle(docs)
total_length=len(docs)
train_length=int(.9*total_length)
train_list=docs[0:train_length]
test_list=docs[train_length:]
print('no of items for train ',len(train_list))
#print(train_list[:10])
print('no of items for test ',len(test_list))


# ***BERT model for the classification***

# %%
def weight_init(m):
    if isinstance(m, torch.nn.Linear):
        print('init of linear is done')
        torch.nn.init.xavier_uniform_(m.weight)
        if m.bias is not None: 
            torch.nn.init.xavier_uniform_(m.bias)


# ***Load tensorboard for the visualization***

# %%
%load_ext tensorboard
%tensorboard --logdir ./


# ***Using Bert Sequence Classification***

# %%
from transformers import BertForSequenceClassification, BertConfig
import torch
import torch.nn as nn
from torch.optim import AdamW 

model = BertForSequenceClassification.from_pretrained(
    "bert-base-uncased", 
    num_labels = 4,   
    output_attentions = False,
    output_hidden_states = False, 
)
print(model)
params = list(model.named_parameters())
optimizer = AdamW(model.parameters(),
                  lr = 2e-5,
                  eps = 1e-8 
                )
#from transformers import get_linear_schedule_with_warmup

# Number of training epochs. The BERT authors recommend between 2 and 4. 
NUM_EPOCHS=4

from torch.optim.lr_scheduler import LambdaLR
writer = SummaryWriter(log_dir='/home/sentrynet/')
total_steps = len(train_list) * NUM_EPOCHS

def lr_lambda(epoch):
    return 0.95 ** epoch  # Example: decay the learning rate by 5% each epoch

# Initialize the scheduler
scheduler = LambdaLR(optimizer, lr_lambda)

model.to('cuda' if torch.cuda.is_available() else 'cpu')

# Create the learning rate scheduler.
#scheduler = get_linear_schedule_with_warmup(optimizer, 
                                            #num_warmup_steps = 0,
                                            #num_training_steps = total_steps)


# %% [markdown]
# ***Train the model***


# %%
for epoch in range(NUM_EPOCHS):
    model.train()
    random.shuffle(train_list)
    for every_trainlist in train_list:
        label1 = every_trainlist.get('Label')
        if label1 is None:
            print("Label not found in:", every_trainlist)
            continue
        
        if isinstance(label1, str):
            label1 = label_mapping.get(label1)
            if label1 is None:
                print("Unknown label:", label1)
                continue

        label1 = torch.tensor([label1]).to('cuda')  # Convert to tensor

        text = every_trainlist.get('Text')
        if text is None or pd.isna(text):
            print("Text not found or is NaN in:", every_trainlist)
            continue  # Skip this iteration
        input_ids = torch.tensor(tokenizer.encode(text, add_special_tokens=True)).unsqueeze(0).to('cuda')

        model.zero_grad()
        
        outputs = model(input_ids, labels=label1)
        loss = outputs.loss
        logits = outputs.logits

        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        scheduler.step()

        _, preds = torch.max(logits, 1)
        accuracy = torch.sum(preds == label1)

        if total_steps % 10 == 0:
            print(f"Step {total_steps} | Loss: {loss.item()} | Accuracy: {accuracy.item()}")
            #with torch.no_grad():
                #writer.add_scalar('training loss', loss.item(), total_steps)
                #writer.add_scalar('training accuracy', accuracy.item(), total_steps)
        
        total_steps += 1

# %% [markdown]
# ***test the model***


# %%
y_actu = []
y_pred = []
model.to('cpu')
model.eval()
tokenizer = BertTokenizer.from_pretrained('bert-base-uncased')

for every_test_list in test_list:
    label1 = every_test_list['Label']
    if label1 is None:
        print("Label not found in:", every_test_list)
        continue

    if isinstance(label1, str):
        label1 = label_mapping.get(label1)
        if label1 is None:
            print("Unknown label:", label1)
            continue

    label1 = torch.tensor([label1])  # Convert to tensor
    text = every_test_list['Text']
    if text is None or pd.isna(text):
        print("Text not found or is NaN in:", every_test_list)
        continue  # Skip this iteration

    input_ids = torch.tensor(tokenizer.encode(text, add_special_tokens=True)).unsqueeze(0)  # Batch size 1
    
    with torch.no_grad():
        outputs = model(input_ids, labels=label1)
        loss = outputs.loss
        logits = outputs.logits
        _, preds = torch.max(logits, 1)
        
        y_actu.append(label1.numpy()[0])
        y_pred.append(preds.numpy()[0])

# %% [markdown]
# ***Confusion Matrics***

# %%
from sklearn.metrics import confusion_matrix
print(confusion_matrix(y_actu, y_pred))


# %%
# Optionally, calculate accuracy or other metrics
from sklearn.metrics import accuracy_score

accuracy = accuracy_score(y_actu, y_pred)
print(f'Accuracy: {accuracy}')

# %% [markdown]
# ***Save the Models***

# %%
torch.save(model, '/home/sentrybot/Desktop/text_emotion_model/model_text.pt')

# %%
#model=torch.load('/content/drive/My Drive/savedModel/model_text.pt')



