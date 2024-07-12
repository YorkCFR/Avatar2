# Provide a generic wrapper for simple langchain LLM interface with a promp and a vectorstore.
from langchain.prompts.prompt import PromptTemplate
from langchain_community.llms import LlamaCpp
from .llm import LLM
import pickle
import time

class LLMLangChain(LLM):
    def __init__(self, model, prompt, format, vectorstore, max_vectors=2, n_ctx=2048, temperature=0, verbose=False, n_gpu_layers=26):
        with open(vectorstore, "rb") as f:
            self.vectorstore = pickle.load(f)
        self._llm = LlamaCpp(model_path=model, temperature=temperature, n_ctx=n_ctx, verbose=verbose,  n_gpu_layers=n_gpu_layers)
        self._prompt = prompt
        self._format = format
        self._max_vectors = max_vectors

    def response(self, text):
        if len(text) < 10:
            return
        docs = self.vectorstore.as_retriever(search_kwargs={"k" : self._max_vectors}).get_relevant_documents(query=text)

        prompt = self._prompt
        for doc in docs:
            prompt = prompt + "\n" + doc.page_content
        prompt = prompt + self._format.format(question=text)
    
        resp = self._llm(prompt)
        return prompt, resp
