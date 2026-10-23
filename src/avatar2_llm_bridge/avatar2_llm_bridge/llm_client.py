import argparse
import re
import sys
from typing import Dict, List, Optional

from llama_cpp import Llama

_THINK_BLOCK_RE = re.compile(r"<think>.*?</think>", re.DOTALL | re.IGNORECASE)
# Disabling these are the model's system prompt's job, so we no longer need to strip them out. But leaving this here in case we want to re-enable it later.
# _EMOJI_RE = re.compile(
#     "["
#     "\U0001F300-\U0001FAFF"
#     "\U00002600-\U000026FF"
#     "\U00002700-\U000027BF"
#     "\U0001F1E6-\U0001F1FF"
#     "\U0000FE0F"
#     "\U0000200D"
#     "]+",
#     flags=re.UNICODE,
# )


def strip_thinking(text: str) -> str:
    """Remove any <think>...</think> reasoning block. 
    This is the second line of defense only. 
    _render_chatml_prompt() is what actually prevents the model from generating one in the first place."""
    return _THINK_BLOCK_RE.sub("", text).strip()

def strip_emojis(text: str) -> str:
    """Strip emoji/pictograph glyphs. 
    This is the second line of defense only.
    system_prompt is what actually tells the model not to produce them."""
    return _EMOJI_RE.sub("", text).strip()


def _render_chatml_prompt(messages: List[Dict[str, str]], enable_thinking: bool) -> str:
    """Hand-rolled ChatML rendering - see module docstring for why this
    bypasses llama-cpp-python's automatic chat-template handling."""
    parts = []
    for m in messages:
        parts.append(f"<|im_start|>{m['role']}\n{m['content']}<|im_end|>\n")
    parts.append("<|im_start|>assistant\n")
    parts.append("<think>\n\n</think>\n\n" if not enable_thinking else "<think>\n")
    return "".join(parts)


class LocalLLM:
    def __init__(
        self,
        model_path: str,
        n_ctx: int = 4096,
        n_gpu_layers: int = -1,
        enable_thinking: bool = False,
        verbose: bool = False,
    ):
        self.enable_thinking = enable_thinking
        self.llm = Llama(
            model_path=model_path,
            n_ctx=n_ctx,
            n_gpu_layers=n_gpu_layers,
            verbose=verbose,
        )

    def generate(
        self,
        messages: List[Dict[str, str]],
        max_tokens: int = 512,
        temperature: float = 0.7,
        top_p: float = 0.8,
        top_k: int = 20,
        min_p: float = 0.0,
        presence_penalty: float = 1.5,
        repeat_penalty: float = 1.0,
    ) -> str:
        prompt = _render_chatml_prompt(messages, self.enable_thinking)
        out = self.llm.create_completion(
            prompt=prompt,
            max_tokens=max_tokens,
            temperature=temperature,
            top_p=top_p,
            top_k=top_k,
            min_p=min_p,
            presence_penalty=presence_penalty,
            repeat_penalty=repeat_penalty,
            stop=["<|im_end|>", "<|im_start|>"],
        )
        text = out["choices"][0]["text"] or ""
        # Since the model strips out the emojis using system propmt, we just do a double sweep for <think>...</think> blocks only
        # return strip_emojis(strip_thinking(text))
        return strip_thinking(text)


def _main():
    parser = argparse.ArgumentParser(description="Standalone local LLM smoke test")
    parser.add_argument("model_path", help="path to a GGUF file")
    parser.add_argument(
        "prompt", nargs="?", default="Introduce yourself as a friendly owl in one sentence."
    )
    parser.add_argument(
        "--system", default="You are a friendly owl avatar. Answer briefly and warmly."
    )
    parser.add_argument("--n-ctx", type=int, default=4096)
    parser.add_argument("--n-gpu-layers", type=int, default=-1)
    parser.add_argument(
        "--enable-thinking", action="store_true", help="allow the model to run a thinking pass"
    )
    parser.add_argument("--max-tokens", type=int, default=512)
    parser.add_argument("--temperature", type=float, default=0.7)
    parser.add_argument("--top-p", type=float, default=0.8)
    parser.add_argument("--top-k", type=int, default=20)
    parser.add_argument("--min-p", type=float, default=0.0)
    parser.add_argument("--presence-penalty", type=float, default=1.5)
    parser.add_argument("--repeat-penalty", type=float, default=1.0)
    args = parser.parse_args()

    llm = LocalLLM(
        model_path=args.model_path,
        n_ctx=args.n_ctx,
        n_gpu_layers=args.n_gpu_layers,
        enable_thinking=args.enable_thinking,
        verbose=True, 
    )

    messages = [
        {"role": "system", "content": args.system},
        {"role": "user", "content": args.prompt},
    ]

    print(f"\nenable_thinking: {llm.enable_thinking}", file=sys.stderr)
    print(f"prompt: {args.prompt}\n", file=sys.stderr)
    reply = llm.generate(
        messages,
        max_tokens=args.max_tokens,
        temperature=args.temperature,
        top_p=args.top_p,
        top_k=args.top_k,
        min_p=args.min_p,
        presence_penalty=args.presence_penalty,
        repeat_penalty=args.repeat_penalty,
    )
    print(reply)


if __name__ == "__main__":
    _main()