from langchain.prompts.prompt import PromptTemplate
#from langchain.llms import LlamaCpp
from langchain_community.llms import LlamaCpp
from langchain.chains import ChatVectorDBChain
import pickle
import time


def get_chain(vectorstore):
    global qa_prompt
    llm =LlamaCpp(n_gpu_layers=12, model_path="./WizardLM-7B-uncensored.ggmlv3.q4_1.bin", temperature=0, n_ctx=2048, verbose=True, use_mlock=True)
    qa_chain = ChatVectorDBChain.from_llm(
        llm,
        vectorstore,
        combine_docs_chain_kwargs={'prompt': qa_prompt},
        condense_question_prompt=CONDENSE_QUESTION_PROMPT,
    )
    return qa_chain




if __name__ == "__main__":
    question = "When is the clinic open"
    with open("hearing.pkl", "rb") as f:
        vectorstore = pickle.load(f)
    llm =LlamaCpp(model_path="./dolphin-2.1-mistral-7b.Q5_K_S.gguf", temperature=0, n_ctx=2048, verbose=False,  n_gpu_layers=-1)


    while True:
        print("Waiting for query>")
        question = input()
        start_time = time.time()
        docs = vectorstore.as_retriever(search_kwargs={"k":2}).get_relevant_documents(query=question)
#        print(f"We got {len(docs)} documents from the vectore store")
#        for doc in docs:
#            print(f'|{doc.page_content}|')
        prompt = """<|im_start|>system
           You are an assistant at the Exquisite Hearing Clinic.
           If you don't know the answer, just say "I'm not sure." Don't try to make up an answer.
           Your name is Mary. Use the following pieces of context to answer the user's question.\n """

        for doc in docs:
            prompt = prompt + "\n" + doc.page_content
        prompt = prompt + f"<|im_end|>\n<|im_start|>user\n{question}<|im_end|>\n<|im_start|>assistant"
    
        print(f'prompt is |{prompt}|')
        resp = llm(prompt)
        print(f'({time.time()-start_time}) {resp}')
#    qa_chain = get_chain(vectorstore)
#    chat_history = []
#    print("Chat with your docs!")
#    while True:
#        print("Human:")
#        question = input()
#        result = qa_chain({"question": question, "chat_history": chat_history})
#        chat_history.append((question, result["answer"]))
#        print("AI:")
#        print(result["answer"])
