"""
Entry point for the flythrough application.

Running this script with ``python run.py`` will start the FastAPI
server that powers both the backend API and the frontend UI.  The
application defined in ``backend/app/main.py`` is imported after
adjusting the Python path to include the repository root.
"""

from __future__ import annotations

import sys
from pathlib import Path

import logging
import uvicorn

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)


def main() -> None:
    """Run the Uvicorn server hosting the flythrough application."""
    # Determine the repository root relative to this file and ensure it is on
    # sys.path so that ``backend`` can be imported as a package.
    repo_root = Path(__file__).resolve().parent
    if str(repo_root) not in sys.path:
        sys.path.append(str(repo_root))

    # Import the FastAPI application.  We import inside main() to avoid
    # modifying sys.path at module import time.
    from backend.app.main import app  # type: ignore

    # Start Uvicorn.  Bind to all interfaces on port 8000 by default.
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()