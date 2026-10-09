"""Stable repository/workspace locations, independent of a script's launch cwd."""
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parent
REPO_ROOT = SOURCE_ROOT.parent
WORKSPACE_ROOT = REPO_ROOT.parent


def repo_file(name):
    """Resolve source-evidence references after the repository layout change.

    This does not rewrite archived manifests or accept changed source hashes.
    Historic names identify current source; frozen experiments retain their own
    copies and are not silently re-attested with new code.
    """
    relative = Path(name)
    if relative.anchor or '..' in relative.parts:
        raise ValueError('Expected a repository-relative source reference')
    if relative.parts and relative.parts[0] == 'om6_piper':
        relative = Path(*relative.parts[1:])
    if len(relative.parts) == 1 and relative.suffix == '.py':
        folder = 'tests' if relative.name.startswith('test_') else 'src'
        candidate = REPO_ROOT / folder / relative
        if candidate.is_file():
            return candidate
    return REPO_ROOT / relative


def workspace_file(name):
    relative = Path(name)
    if relative.anchor or '..' in relative.parts:
        raise ValueError('Expected a workspace-relative source reference')
    if relative.parts and relative.parts[0] == 'om6_piper':
        return repo_file(relative)
    return WORKSPACE_ROOT / relative
