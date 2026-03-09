from __future__ import annotations


def color(text: str, code: str) -> str:
    return f"\033[{code}m{text}\033[0m"


def log(msg: str, level: str = "info") -> None:
    prefix_map = {
        "info": color("[eval]", "36"),
        "ok": color("[ok]", "32"),
        "warn": color("[warn]", "33"),
        "error": color("[error]", "31"),
        "task": color("[task]", "35"),
    }
    prefix = prefix_map.get(level, prefix_map["info"])
    print(f"{prefix} {msg}")
