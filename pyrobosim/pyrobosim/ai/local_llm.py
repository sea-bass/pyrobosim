"""Utilities for running local LLM backends."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol


class LocalLLMError(RuntimeError):
    """Raised when the local LLM backend cannot be initialized or used."""


class TextGenerator(Protocol):
    """Simple protocol for text generators (LLMs, stubs, etc.)."""

    def generate(
        self,
        prompt: str,
        *,
        max_tokens: int | None = None,
        temperature: float | None = None,
        stop: list[str] | None = None,
    ) -> str:
        """Generate text given a prompt."""


@dataclass
class LocalLLM(TextGenerator):
    """Thin wrapper around ``llama-cpp-python`` for local inference."""

    model_path: str
    max_ctx_tokens: int = 2048
    n_threads: int | None = None
    n_gpu_layers: int = 0
    _llama: Any = field(init=False, repr=False)

    def __post_init__(self) -> None:
        try:
            from llama_cpp import Llama  # type: ignore
        except ImportError as exc:
            raise LocalLLMError(
                "llama-cpp-python is not installed. Install it with "
                "`pip install llama-cpp-python` (requires a local wheel or internet access)."
            ) from exc

        try:
            self._llama = Llama(
                model_path=self.model_path,
                n_ctx=self.max_ctx_tokens,
                n_threads=self.n_threads,
                n_gpu_layers=self.n_gpu_layers,
            )
        except (
            Exception
        ) as exc:  # pragma: no cover - backend errors are environment specific
            raise LocalLLMError(f"Failed to load local LLM model: {exc}") from exc

    def generate(
        self,
        prompt: str,
        *,
        max_tokens: int | None = None,
        temperature: float | None = None,
        stop: list[str] | None = None,
    ) -> str:
        completion_kwargs: dict[str, Any] = {
            "prompt": prompt,
            "temperature": temperature if temperature is not None else 0.1,
            "stop": stop,
        }
        if max_tokens is not None:
            completion_kwargs["max_tokens"] = max_tokens

        try:
            response = self._llama.create_completion(**completion_kwargs)
        except AttributeError:
            # Recent llama-cpp-python versions expose __call__ instead of create_completion.
            response = self._llama(
                prompt=prompt,
                max_tokens=max_tokens,
                temperature=completion_kwargs["temperature"],
                stop=stop,
            )
        except (
            Exception
        ) as exc:  # pragma: no cover - backend runtime errors are environment specific
            raise LocalLLMError(f"Local LLM generation failed: {exc}") from exc

        try:
            choice = response["choices"][0]
            return str(choice.get("text", "")).strip()
        except (KeyError, IndexError, TypeError) as exc:
            raise LocalLLMError(
                f"Unexpected response format from local LLM: {response}"
            ) from exc
