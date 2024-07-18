from langchain.text_splitter import RecursiveCharacterTextSplitter, CharacterTextSplitter
from langchain.document_loaders import UnstructuredFileLoader
#from langchain_community.document_loaders import UnstructuredFileLoader
#from langchain_community.document_loaders import TextLoader
from langchain.document_loaders import TextLoader
from langchain.vectorstores.faiss import FAISS
#from langchain_community.embeddings import SentenceTransformerEmbeddings
from langchain.embeddings import SentenceTransformerEmbeddings
import pickle

# Load Data
docs = ['/home/walleed/Avatar2/models/hearing_clinic/text/information_about_hearing_clinic.txtx']
all_docs = []
for doc in docs:
    print(f"Loading document {doc}")
    loader = TextLoader(doc)
    all_docs.extend(loader.load())
print(all_docs)
text_splitter = CharacterTextSplitter(chunk_size=2000, chunk_overlap=0)
documents = text_splitter.split_documents(all_docs)

# Load Data to vectorstore
embeddings = SentenceTransformerEmbeddings(model_name="sentence-transformers/all-MiniLM-L6-v2")
print("Embeddings created")
vectorstore = FAISS.from_documents(documents, embeddings)

# Save vectorstore
with open("hearing.pkl", "wb") as f:
    pickle.dump(vectorstore, f)
