"""Tab completion for the shell tabs, done by a helper bash (ROS-free)."""
from __future__ import annotations

import os
import subprocess
from typing import Sequence

TIMEOUT_S = 2.0
BASH_COMPLETION = "/usr/share/bash-completion/bash_completion"

# Arguments: $1 = word index, $2 = current word, $3 = whole line up to cursor,
# $4 = cursor, remaining = the words. Setup files come via OUM_SETUP (newline separated).
_SCRIPT = r'''
while IFS= read -r f; do
  [ -n "$f" ] && [ -r "$f" ] && source "$f" >/dev/null 2>&1
done <<< "$OUM_SETUP"
if [ -r "$OUM_BASH_COMPLETION" ]; then
  source "$OUM_BASH_COMPLETION" >/dev/null 2>&1
fi
cword=$1; cur=$2; line=$3; point=$4
shift 4
words=("$@")
file_fallback() {
  local f
  while IFS= read -r f; do
    if [ -d "$f" ]; then printf '%s/\n' "$f"; else printf '%s\n' "$f"; fi
  done < <(compgen -f -- "$cur")
}
if [ "$cword" -eq 0 ]; then
  compgen -c -- "$cur"
  exit 0
fi
spec=$(complete -p "${words[0]}" 2>/dev/null)
if [ -z "$spec" ] && declare -F _completion_loader >/dev/null 2>&1; then
  _completion_loader "${words[0]}" >/dev/null 2>&1
  spec=$(complete -p "${words[0]}" 2>/dev/null)
fi
func=""
if [[ "$spec" =~ (^|[[:space:]])-F[[:space:]]+([^[:space:]]+) ]]; then
  func=${BASH_REMATCH[2]}
fi
if [ -n "$func" ] && declare -F "$func" >/dev/null 2>&1; then
  COMP_WORDS=("${words[@]}")
  COMP_CWORD=$cword
  COMP_LINE=$line
  COMP_POINT=$point
  COMP_TYPE=9
  COMP_KEY=9
  COMPREPLY=()
  prev=""
  [ "$cword" -gt 0 ] && prev=${words[cword-1]}
  "$func" "${words[0]}" "$cur" "$prev" >/dev/null 2>&1
  if [ "${#COMPREPLY[@]}" -gt 0 ]; then
    printf '%s\n' "${COMPREPLY[@]}"
    exit 0
  fi
fi
file_fallback
'''


def _split(prefix: str) -> tuple[int, list[str]]:
    """Return (start of the word being completed, words up to the cursor)."""
    start = len(prefix)
    while start > 0 and not prefix[start - 1].isspace():
        start -= 1
    words = prefix.split()
    if prefix == "" or prefix[-1].isspace():
        words.append("")
    return start, words


def complete(line: str, cursor: int, cwd: str,
             setup_files: Sequence[str] = ()) -> tuple[int, list[str]]:
    cursor = max(0, min(int(cursor), len(line)))
    prefix = line[:cursor]
    start, words = _split(prefix)
    cur = prefix[start:]
    env = {
        "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
        "HOME": os.environ.get("HOME", "/"),
        "OUM_SETUP": "\n".join(setup_files),
        "OUM_BASH_COMPLETION": BASH_COMPLETION,
    }
    args = ["bash", "--norc", "--noprofile", "-c", _SCRIPT, "oum-complete",
            str(len(words) - 1), cur, prefix, str(cursor), *words]
    try:
        run = subprocess.run(args, cwd=cwd if os.path.isdir(cwd) else None, env=env,
                             capture_output=True, timeout=TIMEOUT_S, stdin=subprocess.DEVNULL)
    except (subprocess.TimeoutExpired, OSError):
        return start, []
    text = run.stdout.decode("utf-8", "replace")
    cands = sorted({c for c in text.split("\n") if c})
    return start, cands
